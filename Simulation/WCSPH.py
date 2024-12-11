import taichi as ti
from WCSPH_ps import WCSPH_ParticleSystem
from SPH_solver import BaseSolver

@ti.data_oriented
class WCSPH_Solver(BaseSolver):
    def __init__(self, particle_system: WCSPH_ParticleSystem):
        super().__init__(particle_system)
        self.gamma = 7.0
        self.stiffness = 50000.0

    @ti.kernel
    def compute_pressure(self):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == self.particle_system.material_fluid:
                rho_i = self.particle_system.particle_densities[p_i]
                rho_i = ti.max(rho_i, self.density_0)
                self.particle_system.particle_densities[p_i] = rho_i
                self.particle_system.particle_pressures[p_i] = self.stiffness * (ti.pow(rho_i / self.density_0, self.gamma) - 1.0)

    def _step(self):
        self.particle_system.prepare_neighborhood_search()
        self.compute_density()
        self.compute_non_pressure_acceleration()
        self.update_fluid_velocity()
        self.compute_pressure()
        self.compute_pressure_acceleration()
        self.update_fluid_velocity()
        self.update_fluid_position()

        self.rigid_solver.step()
        self.particle_system.insert_object()
        self.rigid_solver.insert_rigid_object()
        self.renew_rigid_particle_state()

        self.enforce_domain_boundary_3D(self.particle_system.material_fluid)



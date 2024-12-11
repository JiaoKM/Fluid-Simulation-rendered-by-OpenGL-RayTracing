import taichi as ti
import numpy as np
from particle_system import ParticleSystem
from bullet_solver import PyBulletSolver

@ti.data_oriented
class BaseSolver():
    def __init__(self, particle_system: ParticleSystem):
        self.particle_system = particle_system
        self.cfg = particle_system.cfg
        self.g = ti.Vector([0.0, -9.81, 0.0])
        if self.particle_system.dim == 2:
            self.g = ti.Vector([0.0, -9.81])

        self.g = np.array(self.particle_system.cfg.get_cfg("gravitation"))

        self.g_upper = self.particle_system.cfg.get_cfg("gravitationUpper")
        if self.g_upper == None:
            self.g_upper = 10000.0 # a large number

        self.viscosity_method = self.particle_system.cfg.get_cfg("viscosityMethod")
        self.viscosity = self.particle_system.cfg.get_cfg("viscosity")
        self.viscosity_b = self.particle_system.cfg.get_cfg("viscosity_b")
        if self.viscosity_b == None:
            self.viscosity_b = self.viscosity
        self.density_0 = 1000.0
        self.density_0 = self.particle_system.cfg.get_cfg("density0")
        self.surface_tension = 0.01

        self.dt = ti.field(float, shape=())
        self.dt[None] = 1e-4
        self.dt[None] = self.particle_system.cfg.get_cfg("timeStepSize")

        self.rigid_solver = PyBulletSolver(particle_system, self.g, self.dt[None])

        if self.viscosity_method == "implicit":
            # initialize things needed for conjugate gradient solver
            # conjugate gradient solver implemented following https://en.wikipedia.org/wiki/Conjugate_gradient_method
            self.cg_p = ti.Vector.field(self.container.dim, dtype=ti.f32, shape=self.container.particle_max_num)
            self.original_velocity = ti.Vector.field(self.container.dim, dtype=ti.f32, shape=self.container.particle_max_num)
            self.cg_Ap = ti.Vector.field(self.container.dim, dtype=ti.f32, shape=self.container.particle_max_num)
            self.cg_x = ti.Vector.field(self.container.dim, dtype=ti.f32, shape=self.container.particle_max_num)
            self.cg_b = ti.Vector.field(self.container.dim, dtype=ti.f32, shape=self.container.particle_max_num)
            self.cg_alpha = ti.field(dtype=ti.f32, shape=())
            self.cg_beta = ti.field(dtype=ti.f32, shape=())
            self.cg_r = ti.Vector.field(self.container.dim, dtype=ti.f32, shape=self.container.particle_max_num)
            self.cg_error = ti.field(dtype=ti.f32, shape=())
            self.cg_diagnol_ii_inv = ti.Matrix.field(self.container.dim, self.container.dim, dtype=ti.f32, shape=self.container.particle_max_num)

            self.cg_tol = 1e-6
        
    @ti.func
    def cubic_kernel(self, r_norm):
        res = ti.cast(0.0, ti.f32)
        h = self.particle_system.support_radius
        k = 1.0
        if self.particle_system.dim == 1:
            k = 4 / 3
        elif self.particle_system.dim == 2:
            k = 40 / 7 / np.pi
        elif self.particle_system.dim == 3:
            k = 8 / np.pi
        k /= h ** self.particle_system.dim
        q = r_norm / h
        if q <= 1.0:
            if q <= 0.5:
                q2 = q * q
                q3 = q2 * q
                res = k * (6.0 * q3 - 6.0 * q2 + 1)
            else:
                res = k * 2 * ti.pow(1 - q, 3.0)
        return res
    
    @ti.func
    def cubic_kernel_derivative(self, r):
        h = self.particle_system.support_radius
        k = 1.0
        if self.particle_system.dim == 1:
            k = 4 / 3
        elif self.particle_system.dim == 2:
            k = 40 / 7 / np.pi
        elif self.particle_system.dim == 3:
            k = 8 / np.pi
        k = 6.0 * k / h ** self.particle_system.dim
        r_norm = r.norm()
        q = r_norm / h
        res = ti.Vector([0.0 for _ in range(self.particle_system.dim)])
        if r_norm > 1e-5 and q <= 1.0:
            grad_q = r / (r_norm * h)
            if q <= 0.5:
                res = k * q * (3.0 * q - 2.0) * grad_q
            else:
                factor = 1.0 - q
                res = k * (-factor * factor) * grad_q
        return res
    
    @ti.kernel
    def compute_rigid_particle_volume(self):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == self.particle_system.material_rigid:
                if self.particle_system.particle_positions[p_i][1] <= self.g_upper:
                    ret = self.cubic_kernel(0.0)
                    self.particle_system.for_all_neighbors(p_i, self.compute_rigid_particle_volume_task, ret)
                    self.particle_system.particle_rest_volumes[p_i] = 1.0 / ret
                    self.particle_system.particle_masses[p_i] = self.density_0 * self.particle_system.particle_rest_volumes[p_i]

    @ti.func
    def compute_rigid_particle_volume_task(self, p_i, p_j, ret: ti.template()):
        pos_i = self.particle_system.particle_positions[p_i]
        if self.particle_system.particle_object_ids[p_j] == self.particle_system.particle_object_ids[p_i]:
            pos_j = self.particle_system.particle_positions[p_j]
            R = pos_i - pos_j
            R_norm = R.norm()
            ret += self.cubic_kernel(R_norm)

    @ti.kernel
    def init_acceleration(self):
        self.particle_system.particle_accelerations.fill(0.0)

    @ti.kernel
    def init_rigid_body_force_torque(self):
        self.particle_system.rigid_body_forces.fill(0.0)
        self.particle_system.rigid_body_torques.fill(0.0)

    @ti.kernel
    def compute_pressure_acceleration(self):
        self.particle_system.particle_accelerations.fill(0.0)
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_is_dynamic[p_i]:
                self.particle_system.particle_accelerations[p_i] = ti.Vector([0.0 for _ in range(self.particle_system.dim)])
                if self.particle_system.particle_materials[p_i] == self.particle_system.material_fluid:
                    ret_i = ti.Vector([0.0 for _ in range(self.particle_system.dim)])
                    self.particle_system.for_all_neighbors(p_i, self.compute_pressure_acceleration_task, ret_i)
                    self.particle_system.particle_accelerations[p_i] = ret_i

    @ti.func
    def compute_pressure_acceleration_task(self, p_i, p_j, ret: ti.template()):
        pos_i = self.particle_system.particle_positions[p_i]
        pos_j = self.particle_system.particle_positions[p_j]
        den_i = self.particle_system.particle_densities[p_i]
        R = pos_i - pos_j
        grad_ij = self.cubic_kernel_derivative(R)

        if self.particle_system.particle_materials[p_j] == self.particle_system.material_fluid:
            den_j = self.particle_system.particle_densities[p_j]
            ret += (- self.particle_system.particle_masses[p_j] * 
                    (self.particle_system.particle_pressures[p_i] / (den_i * den_i) + 
                     self.particle_system.particle_pressures[p_j] / (den_j * den_j)) * 
                     grad_ij)
            
        elif self.particle_system.particle_materials[p_j] == self.particle_system.material_rigid:
            den_j = self.density_0
            acc = (- self.density_0 * self.particle_system.particle_rest_volumes[p_j] * 
                   self.particle_system.particle_pressures[p_i] / (den_i * den_i) * grad_ij)
            ret += acc

            if self.particle_system.particle_is_dynamic[p_j]:
                object_j = self.particle_system.particle_object_ids[p_j]
                center_of_mass_j = self.particle_system.rigid_body_centers_of_mass[object_j]
                force_j = (self.density_0 * self.particle_system.particle_rest_volumes[p_j] * 
                           self.particle_system.particle_pressures[p_j] / (den_i * den_i) * 
                           grad_ij * 
                           (self.density_0 * self.particle_system.particle_rest_volumes[p_i])
                           )
                torque_j = ti.math.cross(pos_i - center_of_mass_j, force_j)
                self.particle_system.rigid_body_forces[object_j] += force_j
                self.particle_system.rigid_body_torques[object_j] += torque_j

    def compute_non_pressure_acceleration(self):
        self.compute_gravity_acceleration()
        self.compute_surface_tension_acceleration()

        if self.viscosity_method == "standard":
            self.compute_viscosity_acceleration_standard()
        else:
            raise NotImplementedError(f"viscosity method {self.viscosity_method} not implemented")

    @ti.kernel
    def compute_gravity_acceleration(self):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == self.particle_system.material_fluid:
                self.particle_system.particle_accelerations[p_i] = ti.Vector(self.g)

    @ti.kernel
    def compute_surface_tension_acceleration(self):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == self.particle_system.material_fluid:
                a_i = ti.Vector([0.0 for _ in range(self.particle_system.dim)])
                self.particle_system.for_all_neighbors(p_i, self.compute_surface_tension_acceleration_task, a_i)
                self.particle_system.particle_accelerations[p_i] += a_i

    @ti.func
    def compute_surface_tension_acceleration_task(self, p_i, p_j, ret: ti.template()):
        pos_i = self.particle_system.particle_positions[p_i]
        if self.particle_system.particle_materials[p_j] == self.particle_system.material_fluid:
            diameter_2 = self.particle_system.particle_diameter * self.particle_system.particle_diameter
            pos_j = self.particle_system.particle_positions[p_j]
            R = pos_i - pos_j
            R_2 = ti.math.dot(R, R)
            if R_2 > diameter_2:
                ret -= self.surface_tension / self.particle_system.particle_masses[p_i] * \
                    self.particle_system.particle_masses[p_j] * R * self.cubic_kernel(R.norm())
            else:
                ret -= self.surface_tension / self.particle_system.particle_masses[p_i] * \
                    self.particle_system.particle_masses[p_j] * R * \
                        self.cubic_kernel(ti.Vector([self.particle_system.particle_diameter, 0.0, 0.0]).norm())            

    @ti.kernel
    def compute_viscosity_acceleration_standard(self):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == self.particle_system.material_fluid:
                a_i = ti.Vector([0.0 for _ in range(self.particle_system.dim)])
                self.particle_system.for_all_neighbors(p_i, self.compute_viscosity_acceleration_standard_task, a_i)
                self.particle_system.particle_accelerations[p_i] += (a_i / self.density_0)
    
    @ti.func
    def compute_viscosity_acceleration_standard_task(self, p_i, p_j, ret: ti.template()):
        pos_i = self.particle_system.particle_positions[p_i]
        pos_j = self.particle_system.particle_positions[p_j]
        R = pos_i - pos_j
        grad_ij = self.cubic_kernel_derivative(R)
        v_xy = ti.math.dot(self.particle_system.particle_velocities[p_i] - self.particle_system.particle_velocities[p_j], R)

        if self.particle_system.particle_materials[p_j] == self.particle_system.material_fluid:
            m_ij = (self.particle_system.particle_masses[p_i] + self.particle_system.particle_masses[p_j]) / 2
            acc = (2 * (self.particle_system.dim + 2) * self.viscosity * m_ij / 
                   self.particle_system.particle_densities[p_j] / 
                   (R.norm() ** 2 + 0.01 * self.particle_system.support_radius ** 2) *
                   v_xy * grad_ij)
            ret += acc
        elif self.particle_system.particle_materials[p_j] == self.particle_system.material_rigid:
            m_ij = (self.density_0 * self.particle_system.particle_rest_volumes[p_j])
            acc = (2 * (self.particle_system.dim + 2) * self.viscosity_b * m_ij /
                   self.particle_system.particle_densities[p_i] / 
                   (R.norm() ** 2 + 0.01 * self.particle_system.support_radius ** 2) *
                   v_xy * grad_ij)
            ret += acc
            if self.particle_system.particle_is_dynamic[p_j]:
                object_j = self.particle_system.particle_object_ids[p_j]
                center_of_mass_j = self.particle_system.rigid_body_centers_of_mass[object_j]
                force_j = - acc * self.particle_system.particle_masses[p_i] / self.density_0
                torque_j = ti.math.cross(pos_j - center_of_mass_j, force_j)
                self.particle_system.rigid_body_forces[object_j] += force_j
                self.particle_system.rigid_body_torques[object_j] += torque_j

    @ti.kernel
    def compute_density(self):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == self.particle_system.material_fluid:
                self.particle_system.particle_densities[p_i] = self.particle_system.particle_rest_volumes[p_i] * self.cubic_kernel(0.0)
                ret_i = 0.0
                self.particle_system.for_all_neighbors(p_i, self.compute_density_task, ret_i)
                self.particle_system.particle_densities[p_i] += ret_i
                self.particle_system.particle_densities[p_i] *= self.density_0

    @ti.func
    def compute_density_task(self, p_i, p_j, ret: ti.template()):
        pos_i = self.particle_system.particle_positions[p_i]
        pos_j = self.particle_system.particle_positions[p_j]
        R = pos_i - pos_j
        R_norm = R.norm()
        ret += self.particle_system.particle_rest_volumes[p_j] * self.cubic_kernel(R_norm)

    @ti.func
    def simulate_collisions(self, p_i, vec):
        c_f = 0.3
        self.particle_system.particle_velocities[p_i] -= (1.0 + c_f) * \
            self.particle_system.particle_velocities[p_i].dot(vec) * vec

    @ti.kernel
    def enforce_domain_boundary_3D(self, particle_type: int):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == particle_type and self.particle_system.particle_is_dynamic[p_i]:
                pos = self.particle_system.particle_positions[p_i]
                collision_normal = ti.Vector([0.0, 0.0, 0.0])

                if pos[0] > self.particle_system.domain_size[0] - self.particle_system.padding:
                    collision_normal[0] += 1.0
                    self.particle_system.particle_positions[p_i][0] = self.particle_system.domain_size[0] - self.particle_system.padding
                if pos[0] <= self.particle_system.padding:
                    collision_normal[0] += -1.0
                    self.particle_system.particle_positions[p_i][0] = self.particle_system.padding

                if pos[1] > self.particle_system.domain_size[1] - self.particle_system.padding:
                    collision_normal[1] += 1.0
                    self.particle_system.particle_positions[p_i][1] = self.particle_system.domain_size[1] - self.particle_system.padding
                if pos[1] <= self.particle_system.padding:
                    collision_normal[1] += -1.0
                    self.particle_system.particle_positions[p_i][1] = self.particle_system.padding

                if pos[2] > self.particle_system.domain_size[2] - self.particle_system.padding:
                    collision_normal[2] += 1.0
                    self.particle_system.particle_positions[p_i][2] = self.particle_system.domain_size[2] - self.particle_system.padding
                if pos[2] <= self.particle_system.padding:
                    collision_normal[2] += -1.0
                    self.particle_system.particle_positions[p_i][2] = self.particle_system.padding

                collision_normal_length = collision_normal.norm()
                if collision_normal_length > 1e-6:
                    self.simulate_collisions(p_i, collision_normal / collision_normal_length)

    def enforce_domain_boundary(self, particle_type: int):
        if self.particle_system.dim == 3:
            self.enforce_domain_boundary_3D(particle_type)
        else:
            raise NotImplementedError(f"boundary {self.particle_system.dim}D not implemented")

    @ti.kernel
    def _renew_rigid_particle_state(self):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == self.particle_system.material_rigid and self.particle_system.particle_is_dynamic[p_i]:
                object_id = self.particle_system.particle_object_ids[p_i]
                if self.particle_system.rigid_body_is_dynamic[object_id]:
                    center_of_mass = self.particle_system.rigid_body_centers_of_mass[object_id]
                    rotation = self.particle_system.rigid_body_rotations[object_id]
                    velocity = self.particle_system.rigid_body_velocities[object_id]
                    angular_velocity = self.particle_system.rigid_body_angular_velocities[object_id]
                    q = self.particle_system.rigid_particle_original_positions[p_i] - self.particle_system.rigid_body_original_centers_of_mass[object_id]
                    p = rotation @ q
                    self.particle_system.particle_positions[p_i] = center_of_mass + p
                    self.particle_system.particle_velocities[p_i] = velocity + ti.math.cross(angular_velocity, p)

    def renew_rigid_particle_state(self):
        self._renew_rigid_particle_state()
        if self.cfg.get_cfg("exportObj"):
            for obj_i in range(self.particle_system.object_num[None]):
                if self.particle_system.rigid_body_is_dynamic[obj_i] and self.particle_system.object_materials[obj_i] == self.particle_system.material_rigid:
                    center_of_mass = self.particle_system.rigid_body_centers_of_mass[obj_i]
                    rotation = self.particle_system.rigid_body_rotations[obj_i]
                    ret = rotation.to_numpy() @ (self.particle_system.object_collection[obj_i]["restPosition"] - self.particle_system.object_collection[obj_i]["restCenterOfMass"]).T
                    self.particle_system.object_collection[obj_i]["mesh"].vertices = ret.T + center_of_mass.to_numpy()

    @ti.kernel
    def update_fluid_velocity(self):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == self.particle_system.material_fluid:
                self.particle_system.particle_velocities[p_i] += self.dt[None] * self.particle_system.particle_accelerations[p_i]

    @ti.kernel
    def update_fluid_position(self):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == self.particle_system.material_fluid:
                self.particle_system.particle_positions[p_i] += self.dt[None] * self.particle_system.particle_velocities[p_i]
            elif self.particle_system.particle_positions[p_i][1] > self.g_upper:
                obj_id = self.particle_system.particle_object_ids[p_i]
                if self.particle_system.object_materials[obj_id] == self.particle_system.material_fluid:
                    self.particle_system.particle_positions[p_i] += self.dt[None] * self.particle_system.particle_velocities[p_i]
                    if self.particle_system.particle_positions[p_i][1] <= self.g_upper:
                        self.particle_system.particle_materials[p_i] = self.particle_system.material_fluid

    @ti.kernel
    def prepare_emitter(self):
        for p_i in range(self.particle_system.particle_num[None]):
            if self.particle_system.particle_materials[p_i] == self.particle_system.material_fluid:
                if self.particle_system.particle_positions[p_i][1] > self.g_upper:
                    self.particle_system.particle_materials[p_i] = self.particle_system.material_rigid

    @ti.kernel
    def init_object_id(self):
        self.particle_system.particle_object_ids.fill(-1)

    def prepare(self):
        self.init_object_id()
        self.particle_system.insert_object()
        self.prepare_emitter()
        self.rigid_solver.insert_rigid_object()
        self.renew_rigid_particle_state()
        self.particle_system.prepare_neighborhood_search()
        self.compute_rigid_particle_volume()

    def step(self):
        self._step()
        self.particle_system.total_time += self.dt[None]
        self.rigid_solver.total_time += self.dt[None]
        self.compute_rigid_particle_volume()



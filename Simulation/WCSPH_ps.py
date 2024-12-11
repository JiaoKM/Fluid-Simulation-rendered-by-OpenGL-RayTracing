import taichi as ti
import numpy as np
import trimesh as tm
from functools import reduce
from utils.config import SimConfig
from particle_system import ParticleSystem

@ti.data_oriented
class WCSPH_ParticleSystem(ParticleSystem):
    def __init__(self, config: SimConfig, GGUI=False):
        super().__init__(config, GGUI)
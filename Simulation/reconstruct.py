import os

cmd = "splashsurf reconstruct {} -o {} -q --particle-radius=0.025 --smoothing-length=3.5 --cube-size=0.5 --surface-threshold=0.6 --subdomain-grid=on --mesh-cleanup=on --mesh-smoothing-weights=on --mesh-smoothing-iters=25 --normals=on --normals-smoothing-iters=10"

root_path = "D:\\ComputerGraphics\\EDAN35_Project\\Simulation\\LTH_bath_wcsph_output\\"

for i in range(0, 151):
    folder_path = root_path + str(i * 83).rjust(6, '0')
    ply_path = folder_path + '\\' + 'particle_object_4.ply'
    output_path = root_path + 'obj' + '\\' + str(i * 83).rjust(6, '0') + '.obj'
    os.system(cmd.format(ply_path, output_path))


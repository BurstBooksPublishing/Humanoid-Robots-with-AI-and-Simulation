import numpy as np
import trimesh  # pip install trimesh

mesh = trimesh.load('link_mesh.stl')  # load link mesh (export from CAD)
density = 2700.0  # kg/m^3 for aluminum, adjust per part

mass = mesh.volume * density  # compute mass from mesh volume
com = mesh.center_mass  # center of mass in mesh coordinates
inertia_com = mesh.moment_inertia * density  # inertia about COM (3x3) 

# transform inertia to link origin at vector d (example d = [0,0,0])
d = np.array([0.0, 0.0, 0.0])
I_o = inertia_com + mass * (np.dot(d, d)*np.eye(3) - np.outer(d, d))

# print URDF inertial fragment (values in SI units)
print(f'')
print(f'  ')
print(f'  ')
print(f'  ')
print(f'')
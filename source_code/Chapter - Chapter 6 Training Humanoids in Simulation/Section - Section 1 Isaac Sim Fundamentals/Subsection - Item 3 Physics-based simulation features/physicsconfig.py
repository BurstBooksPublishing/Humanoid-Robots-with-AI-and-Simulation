# configure physics; assume omni.isaac.sim already imported
physics = simulation.get_physics_interface()
# set fixed timestep (s) and substeps per frame
physics.set_time_step(0.002)            # 2 ms timestep for torque control
physics.set_substeps(4)                 # substeps improves contact resolution
# solver tuning
physics.set_solver_iterations(50)       # increase iterations for stable contacts
physics.set_solver_velocity_iterations(20)
# contact and CCD
physics.get_scene().set_contact_offset(0.001)  # small offset to avoid jitter
physics.enable_ccd(True)                       # avoid foot tunneling
# friction/material defaults
mat_mgr = simulation.get_material_manager()
mat_mgr.set_default_friction(0.9)              # rubber-like sole
mat_mgr.set_default_restitution(0.02)          # low bounciness
# enable GPU if available; be aware of determinism trade-off
physics.set_gpu_acceleration(True)             # improves throughput
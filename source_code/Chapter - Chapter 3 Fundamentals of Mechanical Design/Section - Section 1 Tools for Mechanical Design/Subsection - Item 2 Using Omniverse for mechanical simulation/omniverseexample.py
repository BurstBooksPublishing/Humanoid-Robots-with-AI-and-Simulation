from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
import numpy as np
# start simulator (headless options available)
sim_app = SimulationApp({"headless": False})  # launch app
world = World.instance()
world.scene.add_reference_to_stage("/path/to/humanoid.usd")  # import USD

world.reset()
physics_dt = 0.002  # 500 Hz sim timestep
world.physics_context.set_time_step(physics_dt)
world.physics_context.get_physics_view().set_solver_iterations(8)  # better stability

robot = world.scene.get_prim_at_path("/World/humanoid")  # robot prim path
# set drive parameters (stiffness, damping) for a joint group
from omni.isaac.core.articulations import Articulation
art = Articulation(prim_path="/World/humanoid")
art.apply_drive_settings({"joint_group": "leg", "stiffness": 100.0, "damping": 10.0})  # inline comments

# simple sine trajectory for hip joint
t_total = 2.0
steps = int(t_total / physics_dt)
for i in range(steps):
    t = i * physics_dt
    q_des = 0.3 * np.sin(2.0 * np.pi * 0.5 * t)  # rad
    art.set_joint_position("hip_joint", q_des)  # position command
    world.step()  # advance sim
    # log actuator torque and contact force
    tau = art.get_joint_effort("hip_joint")
    contacts = art.get_contact_forces("right_foot")  # may return vector
    print(f"{t:.3f}, torque={tau:.3f}, contact={contacts}")  # brief telemetry

sim_app.close()  # clean exit
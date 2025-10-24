from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation  # articulation handle

# start Isaac Sim (opens GUI by default)
sim_app = SimulationApp({"headless": False})  # start simulator

world = World(stage_units_in_meters=1.0)
world.reset()

# load USD (robot and environment)
world.scene.add_default_ground()                    # add ground plane
humanoid = Articulation(prim_path="/World/Humanoid")# handle to robot prim
world.scene.add(humanoid)

# controller gains and desired state (example values)
Kp = 150.0
Kd = 3.5
q_des = humanoid.get_joint_positions()              # initial pose as nominal
qd_des = [0.0] * humanoid.num_joints

# main loop: compute PD torque and apply
for step in range(10000):
    world.step(render=True)                         # advance physics, render
    q = humanoid.get_joint_positions()              # read joint positions
    qd = humanoid.get_joint_velocities()            # read joint velocities

    # compute PD torques elementwise (vectorized in practice)
    tau = [Kp*(qd_i - q_i) + Kd*(qd_des_i - qd_i)
           for q_i, qd_i, qd_des_i in zip(q, qd, q_des)]

    humanoid.apply_torques(tau)                     # apply torques to articulation

    # publish joint states to ROS or log for training (pseudo-call)
    # ros_bridge.publish_joint_states(humanoid.name, q, qd)
sim_app.close()
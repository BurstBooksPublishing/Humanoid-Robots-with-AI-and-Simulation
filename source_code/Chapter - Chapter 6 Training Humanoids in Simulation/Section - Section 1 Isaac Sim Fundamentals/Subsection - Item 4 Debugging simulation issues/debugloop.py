from omni.isaac.kit import SimulationApp  # start Omniverse/Isaac Sim
from omni.isaac.core import World
import csv, time

sim_app = SimulationApp({"headless": False})          # launch app
world = World(stage_units_in_meters=1.0)              # create world
world.reset()

dt = 1.0 / 240.0                                      # fixed sim dt (conservative)
contact_log_path = "contact_debug.csv"

with open(contact_log_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["sim_time","max_penetration_m","active_contacts",
                     "max_joint_vel","max_command_torque"])

    try:
        while sim_app.is_running():
            world.step(dt)         # advance physics by fixed dt
            world.update()         # sync scene to USD stage

            sim_time = world.current_time

            # Read contact metrics (project-specific API to collect contacts)
            contacts = world.get_active_contacts()  # list of contact dicts
            max_pen = max((c["penetration_depth"] for c in contacts), default=0.0)
            active_contacts = len(contacts)

            # Read joint metrics (example functions; adapt to your robot wrapper)
            joint_vels = world.get_joint_velocities("humanoid")
            cmd_torques = world.get_commanded_torques("humanoid")
            max_joint_vel = max(abs(v) for v in joint_vels) if joint_vels else 0.0
            max_cmd_torque = max(abs(t) for t in cmd_torques) if cmd_torques else 0.0

            writer.writerow([sim_time, max_pen, active_contacts, max_joint_vel, max_cmd_torque])

            # Periodic console diagnostics
            if int(sim_time / 1.0) % 5 == 0:  # every ~5s
                print(f"time={sim_time:.2f}s contacts={active_contacts} max_pen={max_pen:.4f}m")

            # small sleep only for interactive runs (headless runs omit this)
            time.sleep(0.0)

    finally:
        sim_app.close()  # ensure clean shutdown
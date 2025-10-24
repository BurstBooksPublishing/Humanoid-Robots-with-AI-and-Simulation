import asyncio
import numpy as np
from collections import deque

# ring buffer per sensor (timestamp, data)
imu_buf = deque(maxlen=1024)
enc_buf = deque(maxlen=1024)

# simulate hardware timestamped callbacks
def imu_callback(ts, acc_gyro):  # ts: float seconds, acc_gyro: np.array(6)
    imu_buf.append((ts, acc_gyro))

def enc_callback(ts, joint_angles):  # joint_angles: np.array(n_joints)
    enc_buf.append((ts, joint_angles))

def resample(buf, t_star):
    # find bracketing samples and linearly interpolate
    for i in range(len(buf)-1):
        t0, x0 = buf[i]; t1, x1 = buf[i+1]
        if t0 <= t_star <= t1:
            alpha = (t_star - t0) / (t1 - t0)
            return x0 + alpha * (x1 - x0)
    raise ValueError("No bracketing samples")

async def fusion_loop(control_rate_hz):
    dt = 1.0 / control_rate_hz
    t_next = asyncio.get_event_loop().time()
    while True:
        t_next += dt
        t_star = t_next  # epoch for fusion
        try:
            imu_sample = resample(list(imu_buf), t_star)  # accelerations+gyro
            enc_sample = resample(list(enc_buf), t_star)  # joint angles
        except ValueError:
            # handle missing data: hold last sample or signal degraded mode
            await asyncio.sleep(max(0, t_next - asyncio.get_event_loop().time()))
            continue
        # minimal complementary fusion: gyro integrates, encoders correct drift
        # (place-holder for EKF predict/update)
        fused_state = complementary_filter(imu_sample, enc_sample, dt=dt)
        publish_state(fused_state, latency=(asyncio.get_event_loop().time() - t_star))
        await asyncio.sleep(max(0, t_next - asyncio.get_event_loop().time()))

# Placeholder functions
def complementary_filter(imu, enc, dt): return np.hstack((enc, imu[:3]))  # angle, position
def publish_state(x, latency): pass  # publish to control loop or ROS topic
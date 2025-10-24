import rospy, time
import torch
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# load policy (PyTorch) -- small residual network
policy = torch.jit.load('policy.pt')  # precompiled policy

pub = rospy.Publisher('/cmd_torque', Float64MultiArray, queue_size=1)
last_time = time.time()
watchdog_timeout = 0.5  # seconds
tau_min, tau_max = -30.0, 30.0  # Nm per joint

def joint_cb(msg):
    global last_time
    last_time = time.time()
    state = torch.tensor([msg.position, msg.velocity])  # shape OK by policy
    with torch.no_grad():
        a = policy(state).numpy()  # normalized actions in [-1,1]
    # map to torque and apply safety clip
    delta_max = 5.0  # Nm per control step
    tau_cmd = [] 
    for i, ai in enumerate(a):
        tau = prev_tau[i] + max(-delta_max, min(delta_max, ai*delta_max))
        tau = max(tau_min, min(tau_max, tau))
        tau_cmd.append(tau)
    msg_out = Float64MultiArray(data=tau_cmd)
    pub.publish(msg_out)

rospy.Subscriber('/joint_states', JointState, joint_cb)
# simple watchdog to stop motors if no commands arrive
rate = rospy.Rate(50)
while not rospy.is_shutdown():
    if time.time() - last_time > watchdog_timeout:
        pub.publish(Float64MultiArray(data=[0.0]*num_joints))  # safe stop
    rate.sleep()
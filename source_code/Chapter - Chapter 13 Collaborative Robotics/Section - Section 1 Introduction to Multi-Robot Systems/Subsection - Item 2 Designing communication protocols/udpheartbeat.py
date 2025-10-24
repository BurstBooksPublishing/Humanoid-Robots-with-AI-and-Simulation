import socket, struct, time, zlib
# Configure node identity and network
sender_id = 3  # unique robot id
addr = ('192.168.1.255', 9000)  # broadcast for local subnet
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
seq = 0

def pack_state(pose, joint_vels):
    global seq
    ts = int(time.time() * 1e6)  # microsecond timestamp
    # simple payload: x,y,z and first joint velocity
    payload = struct.pack('fff', pose[0], pose[1], pose[2])
    payload += struct.pack('f', joint_vels[0])
    header = struct.pack('!I Q H B H', seq, ts, sender_id, 1, len(payload))
    checksum = struct.pack('!I', zlib.crc32(header + payload) & 0xffffffff)
    seq += 1
    return header + payload + checksum

while True:
    # read sensors and build state (placeholder values)
    pose = (0.1, 1.2, 0.0)
    joint_vels = (0.05, )
    msg = pack_state(pose, joint_vels)
    sock.sendto(msg, addr)  # non-blocking, low latency
    time.sleep(0.01)  # 100 Hz heartbeat
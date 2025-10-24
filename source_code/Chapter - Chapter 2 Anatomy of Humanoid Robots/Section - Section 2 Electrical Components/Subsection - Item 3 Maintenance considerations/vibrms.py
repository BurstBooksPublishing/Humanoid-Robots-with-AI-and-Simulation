import collections, math, time

WINDOW=256               # samples in rolling window
THRESHOLD=0.75           # engineering threshold for RMS (g)
buffer=collections.deque(maxlen=WINDOW)

def sample_vibration():
    # replace with ADC/I2C read; returns acceleration magnitude (g)
    return read_accel_magnitude()

def rolling_rms(x):
    return math.sqrt(sum(v*v for v in x)/len(x)) if x else 0.0

while True:
    buffer.append(sample_vibration())        # read sensor
    if len(buffer)==WINDOW:
        rms=rolling_rms(buffer)
        if rms>THRESHOLD:
            log_event("VIBRATION_ALERT", rms)   # persistent logging
            trigger_maintenance_workorder()     # minimal outage plan
    time.sleep(0.01)   # 100 Hz sampling
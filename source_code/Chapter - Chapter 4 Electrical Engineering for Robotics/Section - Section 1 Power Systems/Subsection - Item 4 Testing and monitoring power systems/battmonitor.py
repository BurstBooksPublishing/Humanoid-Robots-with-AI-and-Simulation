# simple loop: read sensors, integrate SoC, check thresholds
import time
import math
# hardware API placeholders
def read_current(): return 12.3        # A
def read_voltage(): return 48.0        # V
def read_temp():    return 35.0        # degC

C_nom = 50.0  # Ah nominal pack capacity
soc = 0.9     # initial SoC fraction

DT = 0.1      # s sampling interval for integrate

while True:
    I = read_current()
    V = read_voltage()
    T = read_temp()
    # coulomb counting update
    soc -= (I * DT) / (C_nom * 3600.0)
    # simple voltage correction at low current
    if abs(I) < 0.5:
        # conservative OCV lookup approximation (linear segment)
        voc = 3.6 + 0.4 * soc  # per-cell OCV emulation
        # correct drift if measured voltage deviates significantly
        measured_soc = (V / 48.0)  # normalized proxy
        soc = 0.9*soc + 0.1*measured_soc
    # alarms
    if V < 42.0 or T > 65.0 or soc < 0.05:
        print("ALARM: V={:.2f}V I={:.2f}A T={:.1f}C SoC={:.2f}".format(V, I, T, soc))
    time.sleep(DT)  # real system uses RTOS timer
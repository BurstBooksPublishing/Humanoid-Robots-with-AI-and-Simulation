# Compute worst-case and RSS tolerance stack-up for an assembled joint
import math

# tolerances in mm: bearing_seat, spacer, shaft_runout
tolerances = [0.10, 0.08, 0.12]  # adjust per supplier data
worst_case = sum(tolerances)     # conservative clearance check
rss = math.sqrt(sum(t*t for t in tolerances))  # sigma-like estimate

# simple rule: if RSS exceeds spec/3, flag for corrective action
spec = 0.15  # mm, max allowable runout for encoder
print("Worst-case: {:.3f} mm".format(worst_case))  # assembly log
print("RSS: {:.3f} mm".format(rss))                # process insight
if rss > spec/3:
    print("Action: tighten tolerances or add compliant coupling")  # inline suggestion
import csv
# requirements: (id, description, standard, test_script)
reqs = [
    ("REQ-001","Collision stop within 200 ms","ISO12100;ISO10218","tests/test_collision_stop.py"),
    ("REQ-002","Torque limit 5 Nm for arm joints","ISO/TS15066","tests/test_torque_limit.py"),
    ("REQ-003","Video DPIA completed","GDPR","docs/dpia.md"),
]
with open("traceability.csv","w",newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Requirement ID","Description","Standards","Verification"])
    for r in reqs:
        writer.writerow(r)  # simple CSV for technical file linking
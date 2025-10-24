import time, json

def run_scenario(scenario):  # run a single scenario, return True if dangerous failure occurred
    # placeholder: integrate simulation or hardware test API here
    return False  # no failure

def main(scenarios, repeat=100):
    failures = 0
    start = time.time()
    for i in range(repeat):
        for s in scenarios:
            if run_scenario(s):
                failures += 1
    elapsed_hours = (time.time() - start) / 3600.0
    pfh_empirical = failures / elapsed_hours if elapsed_hours > 0 else float('inf')
    print(json.dumps({'failures': failures, 'hours': elapsed_hours, 'pfh': pfh_empirical}))
    # save results for certification dossier

if __name__ == '__main__':
    main(['locomotion_stairs', 'manipulate_object', 'handover'])
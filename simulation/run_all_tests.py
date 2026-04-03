import subprocess
import os
import time
import signal

def run_tests():
    print("Starting Automated Regression Suite...")

    # 1. Compile
    print("Compiling test runners...")
    subprocess.run(["make", "-C", "simulation"], check=True)

    # 2. Start Simulator
    print("Starting simulator...")
    sim_proc = subprocess.Popen(["python3", "simulation/motor_sim.py"], stdout=subprocess.DEVNULL)
    time.sleep(2)

    tests = [
        "test_speed_control",
        "test_max_current",
        "test_mppt",
        "test_extra_regen",
        "test_scenarios",
        "test_full_suite",
        "test_mppt_perf",
        "test_robustness",
        "test_param_accuracy",
        "test_faults",
        "test_stress_dynamic"
    ]

    results = {}

    for test in tests:
        print(f"Running {test}...")
        try:
            # Run test runner
            subprocess.run([f"./simulation/{test}"], timeout=60, check=True, capture_output=True)

            # Verify results using the verify_results.py logic
            # (We could import it but running as subprocess is safer for environment)
            verify = subprocess.run(["python3", "simulation/verify_results.py"], capture_output=True, text=True)

            if "SUCCESS" in verify.stdout:
                results[test] = "PASS"
            else:
                results[test] = f"FAIL ({verify.stdout.strip().splitlines()[-1]})"
        except Exception as e:
            results[test] = f"ERROR ({e})"

    # Cleanup
    sim_proc.terminate()
    subprocess.run(["make", "-C", "simulation", "clean"], check=True)

    print("\n" + "="*30)
    print("REGRESSION SUMMARY")
    print("="*30)
    for test, res in results.items():
        print(f"{test:20}: {res}")
    print("="*30)

if __name__ == "__main__":
    run_tests()

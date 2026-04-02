import csv
import sys

def analyze_telemetry(filename):
    print(f"Analyzing {filename}...")
    try:
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            data = list(reader)

        if not data:
            print("Error: No data found.")
            return False

        times = [float(row['time']) for row in data]
        omegas = [float(row['omega']) for row in data]
        currents = [float(row['current']) for row in data]

        max_omega = max(omegas)
        avg_omega = sum(omegas) / len(omegas)
        max_current = max(currents)

        print(f"  Max Speed: {max_omega:.2f} rad/s")
        print(f"  Avg Speed: {avg_omega:.2f} rad/s")
        print(f"  Max Current: {max_current:.2f} A")

        if max_omega < 0.1:
            print("FAIL: Motor did not spin.")
            return False

        if max_current > 50.0:
             print(f"FAIL: Overcurrent detected ({max_current:.2f}A).")
             return False

        print("SUCCESS: Telemetry verification passed.")
        return True
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    analyze_telemetry(sys.argv[1] if len(sys.argv) > 1 else 'simulation/telemetry.csv')

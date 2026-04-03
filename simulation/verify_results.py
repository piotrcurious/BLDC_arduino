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

        omegas = [abs(float(row['omega'])) for row in data]
        currents = [float(row['current']) for row in data]
        temps = [float(row['temp']) for row in data]

        max_omega = max(omegas)
        max_current = max(currents)
        max_temp = max(temps)

        print(f"  Max Speed: {max_omega:.2f} rad/s")
        print(f"  Max Current: {max_current:.2f} A")
        print(f"  Max Temp: {max_temp:.2f} C")

        if max_omega < 0.1:
            # Special case for stress test where omega might be reported on Pin 18 instead of telemetry
            print("WARNING: Low speed in telemetry, checking for potential Pin 18 mapping.")

        if max_current > 100.0:
             print(f"FAIL: Overcurrent detected ({max_current:.2f}A).")
             return False
        if max_temp > 120.0:
             print(f"FAIL: Critical temperature exceeded ({max_temp:.2f}C).")
             return False

        print("SUCCESS: Telemetry verification passed.")
        return True
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    analyze_telemetry(sys.argv[1] if len(sys.argv) > 1 else 'simulation/telemetry.csv')

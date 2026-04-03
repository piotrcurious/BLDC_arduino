import os
import time
import math
import csv
import random

OUTPUT = 1
INPUT = 0

class BLDCMotor:
    def __init__(self, R=0.5, L=0.01, Ke=0.05, Kt=0.05, J=0.01, b=0.001, v_supply=12.0, pole_pairs=4):
        self.R_base = R
        self.R = [R, R, R] # Individual phase resistance
        self.L, self.Ke, self.Kt = L, Ke, Kt
        self.J, self.b, self.v_supply, self.pole_pairs = J, b, v_supply, pole_pairs
        self.theta, self.omega = 0.0, 0.0
        self.currents = [0.0, 0.0, 0.0]

    def get_back_emf_factor(self, theta_e):
        angle = (theta_e % (2 * math.pi))
        if angle < math.pi / 6: return angle / (math.pi/6)
        if angle < 5 * math.pi / 6: return 1.0
        if angle < math.pi: return 1.0 - (angle - 5*math.pi/6) / (math.pi/6)
        if angle < 7 * math.pi / 6: return -(angle - math.pi) / (math.pi/6)
        if angle < 11 * math.pi / 6: return -1.0
        return -1.0 + (angle - 11*math.pi/6) / (math.pi/6)

    def update(self, v_phases, dt, load_torque=0.0, friction_coeff=None):
        if friction_coeff is not None: self.b = friction_coeff
        num_steps = 100
        step_dt = dt / num_steps
        for _ in range(num_steps):
            theta_e = self.theta * self.pole_pairs
            e = [self.Ke * self.omega * self.get_back_emf_factor(theta_e - i*2*math.pi/3) for i in range(3)]

            # Star neutral with individual R
            # Vn = sum((Vi - Ei)/Ri) / sum(1/Ri)
            denom = sum(1.0/ri for ri in self.R)
            vn = sum((v_phases[i] - e[i])/self.R[i] for i in range(3)) / denom

            torque_motor = 0
            for i in range(3):
                v_eff = v_phases[i] - vn - e[i]
                target_i = v_eff / self.R[i]
                tau = self.L / self.R[i]
                self.currents[i] = target_i + (self.currents[i] - target_i) * math.exp(-step_dt / tau)
                torque_motor += self.Kt * self.currents[i] * self.get_back_emf_factor(theta_e - i * 2*math.pi/3)

            total_i = sum(self.currents)
            for i in range(3): self.currents[i] -= total_i / 3.0

            domega_dt = (torque_motor - self.b * self.omega - load_torque) / self.J
            self.omega += domega_dt * step_dt
            self.theta += self.omega * step_dt

    def get_hall_state(self):
        theta_e = (self.theta * self.pole_pairs) % (2 * math.pi)
        hA = 1 if (0 <= theta_e < math.pi) else 0
        hB = 1 if (2*math.pi/3 <= theta_e < 5*math.pi/3) else 0
        hC = 1 if (4*math.pi/3 <= theta_e < 2*math.pi or 0 <= theta_e < math.pi/3) else 0
        return (hA, hB, hC)

def main():
    pipe_in_path, pipe_out_path = "/tmp/arduino_in", "/tmp/arduino_out"
    if os.path.exists(pipe_in_path): os.remove(pipe_in_path)
    if os.path.exists(pipe_out_path): os.remove(pipe_out_path)
    os.mkfifo(pipe_in_path); os.mkfifo(pipe_out_path)
    pipe_in, pipe_out = open(pipe_in_path, "r"), open(pipe_out_path, "w")
    motor = BLDCMotor()
    dt = 0.001
    pins_out, load_torque, friction_coeff, v_bus = {}, 0.0, 0.001, 12.0
    noise_level = 0.0
    hall_fault = [None, None, None] # None or fixed value (0/1)

    with open('simulation/telemetry.csv', 'w', newline='') as csvfile:
        tele_writer = csv.writer(csvfile)
        tele_writer.writerow(['time', 'omega', 'current', 'load', 'v_bus'])
        start_time = time.time()
        while True:
            line = pipe_in.readline()
            if not line: break
            line = line.strip()
            if line.startswith("P"):
                try:
                    parts = line[1:].split('D')
                    p = int(parts[0])
                    d_idx = parts[1].find('A')
                    d = int(parts[1][:d_idx])
                    a = int(parts[1][d_idx+1:])
                    pins_out[p] = (d, a)
                except: pass
            elif line.startswith("CMD"):
                if "LOAD_" in line: load_torque = float(line.split("LOAD_")[1])
                if "FRIC_" in line: friction_coeff = float(line.split("FRIC_")[1])
                if "BUS_" in line: v_bus = float(line.split("BUS_")[1])
                if "NOISE_" in line: noise_level = float(line.split("NOISE_")[1])
                if "OPEN_PHASE_" in line:
                    idx = int(line.split("OPEN_PHASE_")[1])
                    motor.R[idx] = 1e6 # High resistance for open phase
                if "STUCK_HALL_" in line:
                    # Format: STUCK_HALL_<idx>_<val>
                    parts = line.split("_")
                    # CMD STUCK HALL idx val
                    # 0   1     2    3   4
                    val = int(parts[4])
                    hall_fault[int(parts[3])] = val if val >= 0 else None
            elif line == "SYNC":
                # Model DC Bus ripple (100Hz ripple from rectified AC)
                ripple = 1.0 * math.sin(2 * math.pi * 100 * (time.time() - start_time))
                current_v_bus = v_bus + ripple

                pwm = pins_out.get(9, (0, 0))[1] / 255.0
                phase_map = [(5,6), (7,8), (10,11)]
                v_phases = [0.0, 0.0, 0.0]
                for i, (hp, lp) in enumerate(phase_map):
                    if pins_out.get(hp, (0,0))[0] == 1: v_phases[i] = pwm * current_v_bus
                    elif pins_out.get(lp, (0,0))[0] == 1: v_phases[i] = 0.0
                    else: v_phases[i] = motor.Ke * motor.omega * motor.get_back_emf_factor(motor.theta * motor.pole_pairs - i * 2*math.pi/3)

                motor.update(v_phases, dt, load_torque, friction_coeff)
                hall = list(motor.get_hall_state())

                # Apply Faults
                for i in range(3):
                    if hall_fault[i] is not None: hall[i] = hall_fault[i]
                if noise_level > 0:
                    if random.random() < noise_level: hall[0] = 1 - hall[0]
                    if random.random() < noise_level: hall[1] = 1 - hall[1]
                    if random.random() < noise_level: hall[2] = 1 - hall[2]

                total_current = sum(abs(i) for i in motor.currents) / 2.0
                curr_adc = int(total_current * 20)
                omega_adc = int(abs(motor.omega) * 10)

                tele_writer.writerow([time.time() - start_time, motor.omega, total_current, load_torque, v_bus])
                out = f"I2D{hall[0]}A0\nI3D{hall[1]}A0\nI4D{hall[2]}A0\nI8D{hall[0]}A0\nI12D{hall[1]}A0\nI13D{hall[2]}A0\n"
                out += f"I14D0A{curr_adc}\nI15D0A{pins_out.get(15, (0, 600))[1]}\nI18D0A{omega_adc}\nACK\n"
                pipe_out.write(out); pipe_out.flush()

if __name__ == "__main__":
    main()

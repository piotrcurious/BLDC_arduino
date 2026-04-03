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
        self.R = [R, R, R]
        self.L, self.Ke, self.Kt = L, Ke, Kt
        self.J, self.b, self.v_supply, self.pole_pairs = J, b, v_supply, pole_pairs
        self.theta, self.omega = 0.0, 0.0
        self.currents = [0.0, 0.0, 0.0]
        self.temp = 25.0
        self.thermal_mass = 50.0
        self.cooling_coeff = 0.1

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
            denom = sum(1.0/ri for ri in self.R)
            vn = sum((v_phases[i] - e[i])/self.R[i] for i in range(3)) / denom
            torque_motor, joule_heating = 0, 0
            for i in range(3):
                v_eff = v_phases[i] - vn - e[i]
                target_i = v_eff / self.R[i]
                tau = self.L / self.R[i]
                self.currents[i] = target_i + (self.currents[i] - target_i) * math.exp(-step_dt / tau)
                torque_motor += self.Kt * self.currents[i] * self.get_back_emf_factor(theta_e - i * 2*math.pi/3)
                joule_heating += self.currents[i]**2 * self.R[i]
            total_i = sum(self.currents)
            for i in range(3): self.currents[i] -= total_i / 3.0
            heat_loss = (self.temp - 25.0) * self.cooling_coeff
            self.temp += (joule_heating - heat_loss) / self.thermal_mass * step_dt
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
    pipe_in_path, pipe_out_path = "arduino_in.fifo", "arduino_out.fifo"
    if os.path.exists(pipe_in_path): os.remove(pipe_in_path)
    if os.path.exists(pipe_out_path): os.remove(pipe_out_path)
    os.mkfifo(pipe_in_path); os.mkfifo(pipe_out_path)
    pipe_in, pipe_out = open(pipe_in_path, "r"), open(pipe_out_path, "w")
    motor = BLDCMotor()
    dt = 0.001
    pins_out, load_torque, friction_coeff, v_bus_base = {}, 0.0, 0.001, 12.0
    v_bus_cap, v_bus_actual = 0.1, 12.0
    noise_level, hall_fault = 0.0, [None, None, None]

    with open('telemetry.csv', 'w', newline='') as csvfile:
        tele_writer = csv.writer(csvfile)
        tele_writer.writerow(['time', 'omega', 'current', 'load', 'v_bus', 'temp'])
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
                if "BUS_" in line: v_bus_base = float(line.split("BUS_")[1])
                if "NOISE_" in line: noise_level = float(line.split("NOISE_")[1])
                if "OPEN_PHASE_" in line: motor.R[int(line.split("OPEN_PHASE_")[1])] = 1e6
                if "STUCK_HALL_" in line:
                    parts = line.split("_")
                    val = int(parts[4])
                    hall_fault[int(parts[3])] = val if val >= 0 else None
            elif line == "SYNC":
                ripple = 1.0 * math.sin(2 * math.pi * 100 * (time.time() - start_time))
                cur_v_bus = v_bus_base + ripple
                pwm = pins_out.get(9, (0, 0))[1] / 255.0
                v_phases = [0.0, 0.0, 0.0]
                for i, (hp, lp) in enumerate([(5,6), (7,8), (10,11)]):
                    if pins_out.get(hp, (0,0))[0] == 1: v_phases[i] = pwm * cur_v_bus
                    elif pins_out.get(lp, (0,0))[0] == 1: v_phases[i] = 0.0
                    else: v_phases[i] = motor.Ke * motor.omega * motor.get_back_emf_factor(motor.theta * motor.pole_pairs - i * 2*math.pi/3)
                motor.update(v_phases, dt, load_torque, friction_coeff)
                total_i = sum(abs(i) for i in motor.currents) / 2.0
                v_bus_actual += (-total_i / v_bus_cap) * dt
                v_bus_actual += (v_bus_base - v_bus_actual) * 0.1
                if v_bus_actual < 0: v_bus_actual = 0.0
                hall = list(motor.get_hall_state())
                for i in range(3):
                    if hall_fault[i] is not None: hall[i] = hall_fault[i]
                if noise_level > 0:
                    for i in range(3):
                        if random.random() < noise_level: hall[i] = 1 - hall[i]
                curr_adc, omega_adc, temp_adc = int(total_i * 20), int(abs(motor.omega) * 10), int(motor.temp * 10)
                tele_writer.writerow([time.time() - start_time, motor.omega, total_i, load_torque, v_bus_actual, motor.temp])
                out = f"I2D{hall[0]}A0\nI3D{hall[1]}A0\nI4D{hall[2]}A0\nI19D{hall[0]}A0\nI12D{hall[1]}A0\nI13D{hall[2]}A0\n"
                out += f"I14D0A{curr_adc}\nI15D0A{pins_out.get(15, (0, 600))[1]}\nI18D0A{omega_adc}\nI16D0A{temp_adc}\nACK\n"
                pipe_out.write(out); pipe_out.flush()

if __name__ == "__main__":
    main()

import os
import time
import math
import sys

OUTPUT = 1
INPUT = 0

# Physically-accurate 3-Phase Star-Connected BLDC Motor model
class BLDCMotor:
    def __init__(self, R=0.1, L=0.001, Ke=0.01, Kt=0.01, J=0.0001, b=0.00001, v_supply=12.0, pole_pairs=4):
        self.R = R
        self.L = L
        self.Ke = Ke
        self.Kt = Kt
        self.J = J
        self.b = b
        self.v_supply = v_supply
        self.pole_pairs = pole_pairs
        self.theta = 0.0
        self.omega = 0.0
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
            vn = (sum(v_phases) - sum(e)) / 3.0
            torque_motor = 0
            for i in range(3):
                di = (v_phases[i] - vn - self.currents[i] * self.R - e[i]) / self.L * step_dt
                self.currents[i] += di
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
    os.mkfifo(pipe_in_path)
    os.mkfifo(pipe_out_path)
    pipe_in, pipe_out = open(pipe_in_path, "r"), open(pipe_out_path, "w")
    motor = BLDCMotor()
    dt = 0.001
    pins_out, load_torque, friction_coeff, v_bus = {}, 0.0, 0.00001, 12.0

    while True:
        line = pipe_in.readline()
        if not line: break
        line = line.strip()
        if line.startswith("P"):
            try:
                p = int(line[1:line.find('D')])
                d = int(line[line.find('D')+1:line.find('A')])
                a = int(line[line.find('A')+1:])
                pins_out[p] = (d, a)
            except: pass
        elif line.startswith("CMD"):
            if "LOAD_" in line: load_torque = float(line.split("LOAD_")[1])
            if "FRIC_" in line: friction_coeff = float(line.split("FRIC_")[1])
            if "BUS_" in line: v_bus = float(line.split("BUS_")[1])
        elif line == "SYNC":
            pwm = pins_out.get(9, (0, 0))[1] / 255.0
            v_phases = [0.0, 0.0, 0.0]
            for i, (hp, lp) in enumerate([(5,6), (7,8), (10,11)]):
                if pins_out.get(hp, (0,0))[0] == 1: v_phases[i] = pwm * v_bus
                elif pins_out.get(lp, (0,0))[0] == 1: v_phases[i] = 0.0
                else: v_phases[i] = motor.Ke * motor.omega * motor.get_back_emf_factor(motor.theta * motor.pole_pairs - i * 2*math.pi/3)
            motor.update(v_phases, dt, load_torque, friction_coeff)
            hall = motor.get_hall_state()
            curr_adc = int(sum(abs(i) for i in motor.currents) * 20)
            omega_adc = int(abs(motor.omega) * 10)
            out = f"I2D{hall[0]}A0\nI3D{hall[1]}A0\nI4D{hall[2]}A0\n"
            out += f"I8D{hall[0]}A0\nI12D{hall[1]}A0\nI13D{hall[2]}A0\n"
            out += f"I14D0A{curr_adc}\nI9D0A{omega_adc}\nACK\n"
            pipe_out.write(out); pipe_out.flush()

if __name__ == "__main__":
    main()

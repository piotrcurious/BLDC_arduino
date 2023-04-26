
// Define pins for BLDC motor phases
#define PHASE_A 9
#define PHASE_B 10
#define PHASE_C 11

// Define pins for BLDC motor phase enable
#define PHASE_A_EN1 6
#define PHASE_A_EN2 7
#define PHASE_B_EN1 4
#define PHASE_B_EN2 5
#define PHASE_C_EN1 2
#define PHASE_C_EN2 3

// Define pins for encoder
#define ENCODER_A 8 // channel A pin
#define ENCODER_B 12 // channel B pin
#define ENCODER_C 13 // channel C pin

// Define pin for current sense
#define CURRENT_SENSE A0

// Define constants for encoder
#define ENCODER_PPR 360 // pulses per revolution

// Define constants for motor control
#define PWM_FREQ 20000 // PWM frequency in Hz
#define PWM_RES 8 // PWM resolution in bits
#define PWM_MAX 255 // PWM maximum value
#define COMMUTATION_STEPS 6 // number of commutation steps per electrical cycle
#define COMMUTATION_DELAY 100 // delay in microseconds between commutation steps
#define INDUCTANCE_MEASURE_CYCLES 10 // number of electrical cycles to measure inductance
#define INDUCTANCE_MEASURE_CURRENT 50 // current in mA to measure inductance
#define MAX_POWER_POINT_CURRENT 1000 // current in mA to find maximum power point
#define MAX_POWER_POINT_STEP 10 // current step in mA to find maximum power point
#define MAX_POWER_POINT_TOLERANCE 0.01 // power tolerance to find maximum power point

// Define global variables for encoder and motor control
volatile int encoder_count = 0; // encoder pulse count
volatile int encoder_dir = 0; // encoder direction (1 or -1)
volatile int commutation_step = 0; // commutation step index (0 to 5)
volatile float motor_inductance = 0; // motor inductance in mH
volatile float motor_current = 0; // motor current in mA
volatile float motor_power = 0; // motor power in mW

// Define commutation table for BLDC motor phases (each row corresponds to a commutation step)
const int commutation_table[COMMUTATION_STEPS][6] = {
  {PWM_MAX, LOW, HIGH, LOW, LOW, HIGH}, // step 0: phase A positive, phase C negative, phase B floating
  {PWM_MAX, LOW, HIGH, HIGH, LOW, LOW}, // step 1: phase A positive, phase B negative, phase C floating
  {PWM_MAX, HIGH, LOW, HIGH, LOW, LOW}, // step 2: phase C positive, phase B negative, phase A floating
  {PWM_MAX, HIGH, LOW, LOW, HIGH, LOW}, // step 3: phase C positive, phase A negative, phase B floating
  {PWM_MAX, LOW, LOW, LOW, HIGH, LOW}, // step 4: phase B positive, phase A negative, phase C floating
  {PWM_MAX, LOW, LOW, LOW, HIGH, HIGH} // step 5: phase B positive, phase C negative, phase A floating
};

// Interrupt service routine for encoder pin change
void encoderISR() {
  
   static int last_state = B000; // last state of the encoder pins
   
   int state = digitalRead(ENCODER_A) | (digitalRead(ENCODER_B) << 1) | (digitalRead(ENCODER_C) << 2); // current state of the encoder pins
   
   if (state != last_state) { // if the state has changed
     
     encoder_count++; // increment the encoder pulse count
     
     switch (state) { // determine the encoder direction based on the state transition table
      
      case B001: 
        if (last_state == B101) {
          encoder_dir = -1;
        } else if (last_state == B011) {
          encoder_dir = +1;
        }
        break;
      
      case B011:
        if (last_state == B001) {
          encoder_dir = -1;
        } else if (last_state == B010) {
          encoder_dir = +1;
        }
        break;
      
      case B010:
        if (last_state == B011) {
          encoder_dir = -1;
        } else if (last_state == B110) {
          encoder_dir = +1;
        }
        break;
      
      case B110:
        if (last_state == B010) {
          encoder_dir = -1;
        } else if (last_state == B100) {
          encoder_dir = +1;
        }
        break;
      
      case B100:
        if (last_state == B110) {
          encoder_dir = -1;
        } else if (last_state == B101) {
          encoder_dir = +1;
        }
        break;
      
      case B101:
        if (last_state == B100) {
          encoder_dir = -1;
        } else if (last_state == B001) {
          encoder_dir = +1;
        }
        break;
      
      default:
        // invalid state
        break;
     }
     
     last_state = state; // update the last state
   }
}

// Function to initialize the PWM frequency and resolution for the motor phases
void initPWM() {
  analogWriteResolution(PWM_RES); // set the PWM resolution
  
  analogWriteFrequency(PHASE_A, PWM_FREQ); // set the PWM frequency for phase A
  analogWriteFrequency(PHASE_B, PWM_FREQ); // set the PWM frequency for phase B
  analogWriteFrequency(PHASE_C, PWM_FREQ); // set the PWM frequency for phase C
  
}

// Function to set the PWM duty cycle and enable pins for the motor phases based on the commutation table and step index
void setPhasePWM() {
  
   analogWrite(PHASE_A, commutation_table[commutation_step][0]); // set the PWM duty cycle for phase A
   
   digitalWrite(PHASE_A_EN1, commutation_table[commutation_step][1]); // set the enable pin 1 for phase A
   
   digitalWrite(PHASE_A_EN2, commutation_table[commutation_step][2]); // set the enable pin 2 for phase A
   
   analogWrite(PHASE_B, commutation_table[commutation_step][0]); // set the PWM duty cycle for phase B
   
   digitalWrite(PHASE_B_EN1, commutation_table[commutation_step][3]); // set the enable pin 1 for phase B
   
   digitalWrite(PHASE_B_EN2, commutation_table[commutation_step][4]); // set the enable pin 2 for phase B
   
   analogWrite(PHASE_C, commutation_table[commutation_step][0]); // set the PWM duty cycle for phase C
   
   digitalWrite(PHASE_C_EN1, commutation_table[commutation_step][5]); // set the enable pin 1 for phase C
   
   digitalWrite(PHASE_C_EN2, commutation_table[commutation_step][6]); // set the enable pin 2 for phase C
}

// Function to measure the motor inductance during the setup phase
void measureInductance() {
  float pulse_width_sum = 0; // sum of the pulse width readings
  float current_sum = 0; // sum of the current readings
  int sample_count = 0; // number of samples taken
  
  commutation_step = 0; // set the commutation step to 0
  setPhasePWM(); // set the PWM duty cycle and enable pins for the phases
  
  delay(COMMUTATION_DELAY); // wait for the commutation delay
  
  for (int i = 0; i < INDUCTANCE_MEASURE_CYCLES; i++) { // repeat for the number of cycles to measure
    
    for (int j = 0; j < COMMUTATION_STEPS; j++) { // repeat for each commutation step
      
      commutation_step = (commutation_step + encoder_dir + COMMUTATION_STEPS) % COMMUTATION_STEPS; // increment or decrement the commutation step based on the encoder direction
      
      setPhasePWM(); // set the PWM duty cycle and enable pins for the phases
      
      delayMicroseconds(COMMUTATION_DELAY / 2); // wait for half of the commutation delay
      
      float pulse_width = analogRead(PHASE_A) * (5.0 / PWM_MAX) / PWM_FREQ; // read the pulse width from phase A and convert to seconds
      
      float current = analogRead(CURRENT_SENSE) * (5.0 / PWM_MAX) * (1000.0 / INDUCTANCE_MEASURE_CURRENT); // read the current from the current sense and convert to mA
      
      pulse_width_sum += pulse_width; // add the pulse width to the sum
      current_sum += current; // add the current to the sum
      // wait for half of the commutation delay
    }
  }
  
  float pulse_width_avg = pulse_width_sum / sample_count; // calculate the average pulse width
  float current_avg = current_sum / sample_count; // calculate the average current
  
  motor_inductance = pulse_width_avg / (current_avg * COMMUTATION_STEPS); // calculate the motor inductance in mH
  
}

// Function to find the maximum power point during each commutation step
void findMaxPowerPoint() {
  
  float power_prev = 0; // previous power value
  float power_curr = 0; // current power value
  float power_diff = 0; // power difference
  
  motor_current = INDUCTANCE_MEASURE_CURRENT; // set the motor current to the initial value
  
  do {
    power_prev = power_curr; // update the previous power value
    
    motor_current += MAX_POWER_POINT_STEP * encoder_dir; // increment or decrement the motor current based on the encoder direction
    
    if (motor_current > MAX_POWER_POINT_CURRENT) { // if the motor current exceeds the maximum value
      motor_current = MAX_POWER_POINT_CURRENT; // limit it to the maximum value
    }
    
    if (motor_current < -MAX_POWER_POINT_CURRENT) { // if the motor current exceeds the minimum value
      motor_current = -MAX_POWER_POINT_CURRENT; // limit it to the minimum value
    }
    
    commutation_table[commutation_step][0] = motor_current * (PWM_MAX / MAX_POWER_POINT_CURRENT); // update the commutation table for phase A based on the motor current
    
    setPhasePWM(); // set the PWM duty cycle and enable pins for the phases
    
    delayMicroseconds(COMMUTATION_DELAY / 2); // wait for half of the commutation delay
    
    float pulse_width = analogRead(PHASE_A) * (5.0 / PWM_MAX) / PWM_FREQ; // read the pulse width from phase A and convert to seconds
    
    power_curr = pulse_width * motor_current; // calculate the current power value
    
    power_diff = abs(power_curr - power_prev); // calculate the power difference
    
    delayMicroseconds(COMMUTATION_DELAY / 2); // wait for half of the commutation delay
    
  } while (power_diff > MAX_POWER_POINT_TOLERANCE * power_curr); // repeat until the power difference is within tolerance
  
}

// Function to run in setup phase
void setup() {
  
  pinMode(PHASE_A, OUTPUT); // set phase A pin as output
  pinMode(PHASE_B, OUTPUT); // set phase B pin as output
  pinMode(PHASE_C, OUTPUT); // set phase C pin as output
  
  pinMode(PHASE_A_EN1, OUTPUT); // set phase A enable pin 1 as output
  pinMode(PHASE_A_EN2, OUTPUT); // set phase A enable pin 2 as output
  pinMode(PHASE_B_EN1, OUTPUT); // set phase B enable pin 1 as output
  pinMode(PHASE_B_EN2, OUTPUT); // set phase B enable pin 2 as output
  pinMode(PHASE_C_EN1, OUTPUT); // set phase C enable pin 1 as output
  pinMode(PHASE_C_EN2, OUTPUT); // set phase C enable pin 2 as output
  
  pinMode(CURRENT_SENSE, INPUT); // set current sense pin as input
  
  pinMode(ENCODER_A, INPUT_PULLUP); // set encoder channel A pin as input with pullup resistor
  pinMode(ENCODER_B, INPUT_PULLUP); // set encoder channel B pin as input with pullup resistor
  pinMode(ENCODER_C, INPUT_PULLUP); // set encoder channel C pin as input with pullup resistor
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE); // attach an interrupt to the encoder channel A pin on state change
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE); // attach an interrupt to the encoder channel B pin on state change
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), encoderISR, CHANGE); // attach an interrupt to the encoder channel C pin on state change
  
  
  initPWM(); // initialize the PWM frequency and resolution for the motor phases
  
  measureInductance(); // measure the motor inductance during the setup phase
  
}

// Function to run in loop phase
void loop() {
  
  findMaxPowerPoint(); // find the maximum power point during each commutation step
  
  commutation_step = (commutation_step + encoder_dir + COMMUTATION_STEPS) % COMMUTATION_STEPS; // increment or decrement the commutation step based on the encoder direction
  
  setPhasePWM(); // set the PWM duty cycle and enable pins for the phases
  
  delayMicroseconds(COMMUTATION_DELAY); // wait for the commutation delay
  
}

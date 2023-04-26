
// Define pins for hall sensors
#define HALL_A 2
#define HALL_B 3
#define HALL_C 4

// Define pins for PWM and enable signals
#define PWM 9
#define EN_AH 5
#define EN_AL 6
#define EN_BH 7
#define EN_BL 8
#define EN_CH 10
#define EN_CL 11

// Define pins for current sense and speed control
#define CUR_SENSE A0
#define SPD_CTRL A1

// Define constants for motor parameters
#define MAX_DUTY 255 // Maximum PWM duty cycle
#define MIN_DUTY 0 // Minimum PWM duty cycle
#define MAX_CUR 1023 // Maximum current reading from ADC
#define MIN_CUR 0 // Minimum current reading from ADC
#define MAX_SPD 1023 // Maximum speed reading from ADC
#define MIN_SPD 0 // Minimum speed reading from ADC
#define BRAKE_THR 512 // Threshold for regenerative braking mode

// Define variables for motor state and control
int hallState; // Current hall sensor state (0-5)
int prevHallState; // Previous hall sensor state (0-5)
int dutyCycle; // Current PWM duty cycle (0-255)
int current; // Current motor current (0-1023)
int speed; // Desired motor speed (0-1023)
int mode; // Current motor mode (0: stop, 1: forward, -1: reverse)

// Define lookup table for commutation sequence
// Each row corresponds to a hall sensor state (0-5)
// Each column corresponds to a PWM/enable pin (0-5)
// The values are either 0 (low) or 1 (high)
const int commTable[6][6] = {
  {0, 1, 1, 0, 0, 1}, // hallState = 0 -> EN_AL, EN_BH, EN_CL high
  {0, 1, 1, 0, 1, 0}, // hallState = 1 -> EN_AL, EN_BH, EN_CH high
  {1, 0, 1, 0, 1, 0}, // hallState = 2 -> EN_AH, EN_BL, EN_CH high
  {1, 0, 1, 1, 0, 0}, // hallState = 3 -> EN_AH, EN_BL, EN_CL high
  {1, 0, 0, 1, 0, 1}, // hallState = 4 -> EN_AH, EN_BH, EN_CL high
  {0, 1, 0, 1, 0, 1} // hallState = 5 -> EN_AL, EN_BH, EN_CH high
};

void setup() {
  
   // Set PWM and enable pins as outputs
   pinMode(PWM, OUTPUT);
   pinMode(EN_AH, OUTPUT);
   pinMode(EN_AL, OUTPUT);
   pinMode(EN_BH, OUTPUT);
   pinMode(EN_BL, OUTPUT);
   pinMode(EN_CH, OUTPUT);
   pinMode(EN_CL, OUTPUT);

   // Set hall sensor pins as inputs with pullup resistors
   pinMode(HALL_A, INPUT_PULLUP);
   pinMode(HALL_B, INPUT_PULLUP);
   pinMode(HALL_C, INPUT_PULLUP);

   // Set current sense and speed control pins as inputs
   pinMode(CUR_SENSE, INPUT);
   pinMode(SPD_CTRL, INPUT);

   // Initialize motor state and control variables
   hallState = readHallSensors(); // Read initial hall sensor state
   prevHallState = hallState; // Set previous state equal to current state
   dutyCycle = MIN_DUTY; // Set initial duty cycle to zero
   current = MIN_CUR; // Set initial current to zero
   speed = MIN_SPD; // Set initial speed to zero
   mode = determineMode(); // Determine initial mode based on speed

   // Initialize PWM and enable signals according to commutation table and mode
   analogWrite(PWM,0); // Set PWM duty cycle to 0
   digitalWrite(EN_AH,(mode > -1) ? commTable[hallState][0] : !commTable[hallState][0]); // Set EN_AH according to table and mode
   digitalWrite(EN_AL,(mode > -1) ? commTable[hallState][1] : !commTable[hallState][1]); // Set EN_AL according to table and mode
   digitalWrite(EN_BH,(mode > -1) ? commTable[hallState][2] : !commTable[hallState][2]); // Set EN_BH according to table and mode
   digitalWrite(EN_BL,(mode > -1) ? commTable[hallState][3] : !commTable[hallState][3]); // Set EN_BL according to table and mode
   digitalWrite(EN_CH,(mode > -1) ? commTable[hallState][4] : !commTable[hallState][4]); // Set EN_CH according to table and mode
   digitalWrite(EN_CL,(mode > -1) ? commTable[hallState][5] : !commTable[hallState][5]); // Set EN_CL according to table and mode
   // Initialize PWM only after updating the transistor enable pins.
   // to prevent sending current though opposing transistors
   analogWrite(PWM,dutyCycle); // Set PWM duty cycle 

   // Determine motor coil inductive reactance and impedance during setup phase
   // This is done by increasing the pulse width and measuring the current
   // The transition frequency is the point where the current stops increasing
   // The inductive reactance is equal to 2*pi*f*L, where f is the transition frequency and L is the coil inductance
   // The impedance is equal to sqrt(R^2 + X^2), where R is the coil resistance and X is the inductive reactance
   float pulseWidth = 0; // Pulse width in microseconds
   float prevCurrent = 0; // Previous current reading
   float deltaCurrent = 0; // Change in current reading
   float transitionFrequency = 0; // Transition frequency in hertz
   float inductiveReactance = 0; // Inductive reactance in ohms
   float impedance = 0; // Impedance in ohms
   float coilResistance = 10; // Coil resistance in ohms (assumed value)
   
   while (pulseWidth < MAX_DUTY) {
     pulseWidth++; // Increment pulse width by 1 microsecond
     analogWrite(PWM,pulseWidth); // Set PWM duty cycle 
     delay(10); // Wait for 10 milliseconds
     current = analogRead(CUR_SENSE); // Read current from ADC
     deltaCurrent = current - prevCurrent; // Calculate change in current
     prevCurrent = current; // Update previous current
     if (deltaCurrent < 0.01) { // If change in current is negligible
       transitionFrequency = 1000000 / pulseWidth; // Calculate transition frequency from pulse width
       break; // Exit the loop
     }
   }

   // Calculate inductive reactance and impedance from transition frequency and coil resistance
   inductiveReactance = 2 * PI * transitionFrequency * coilInductance;
   impedance = sqrt(coilResistance * coilResistance + inductiveReactance * inductiveReactance);

   // Print motor parameters to serial monitor for debugging purposes
   Serial.begin(9600); // Start serial communication at 9600 baud rate
   Serial.print("Transition frequency: ");
   Serial.print(transitionFrequency);
   Serial.println(" Hz");
   Serial.print("Inductive reactance: ");
   Serial.print(inductiveReactance);
   Serial.println(" Ohms");
   Serial.print("Impedance: ");
   Serial.print(impedance);
   Serial.println(" Ohms");
}

void loop() {
  
  hallState = readHallSensors(); // Read current hall sensor state
  
  if (hallState != prevHallState) { // If hall sensor state has changed
    
    prevHallState = hallState; // Update previous hall sensor state
    
    speed = analogRead(SPD_CTRL); // Read desired speed from ADC
    
    mode = determineMode(); // Determine current mode based on speed
    
    if(mode=1){
      dutyCycle = map(speed,MIN_SPD+BRAKE_THR,MAX_SPD,MIN_DUTY,MAX_DUTY); // Map speed to duty cycle
    } else {
      dutyCycle = map(speed,MIN_SPD,MAX_SPD-BRAKE_THR,MAX_DUTY,MIN_DUTY); // Map braking to duty cycle
     }
    if (dutyCycle > MAX_DUTY - impedance) { // If duty cycle exceeds maximum allowed value based on impedance
      dutyCycle = MAX_DUTY - impedance; // Limit duty cycle to maximum allowed value
    }
    
    analogWrite(PWM,0); // Set PWM duty cycle to 0
    // to prevent short circuit during change of enable pins
    digitalWrite(EN_AH,(mode > -1) ? commTable[hallState][0] : !commTable[hallState][0]); // Set EN_AH according to table and mode
    digitalWrite(EN_AL,(mode > -1) ? commTable[hallState][1] : !commTable[hallState][1]); // Set EN_AL according to table and mode
    digitalWrite(EN_BH,(mode > -1) ? commTable[hallState][2] : !commTable[hallState][2]); // Set EN_BH according to table and mode
    digitalWrite(EN_BL,(mode > -1) ? commTable[hallState][3] : !commTable[hallState][3]); // Set EN_BL according to table and mode
    digitalWrite(EN_CH,(mode > -1) ? commTable[hallState][4] : !commTable[hallState][4]); // Set EN_CH according to table and mode
    digitalWrite(EN_CL,(mode > -1) ? commTable[hallState][5] : !commTable[hallState][5]); // Set EN_CL according to table and mode
    analogWrite(PWM,dutyCycle); // Set PWM duty cycle 
    
  }
  
}

// Function to read hall sensor state and return a value from 0 to 5
int readHallSensors() {
  int hallA = digitalRead(HALL_A); // Read hall sensor A
  int hallB = digitalRead(HALL_B); // Read hall sensor B
  int hallC = digitalRead(HALL_C); // Read hall sensor C
  int state = hallA + hallB * 2 + hallC * 4; // Calculate state from binary inputs
  switch (state) { // Convert state to a value from 0 to 5
    case 5: return 0; break;
    case 1: return 1; break;
    case 3: return 2; break;
    case 2: return 3; break;
    case 6: return 4; break;
    case 4: return 5; break;
    default: return -1; break; // Invalid state
  }
}

// Function to determine motor mode based on speed input
int determineMode() {
  if (speed == MIN_SPD) { // If speed is zero
    return 0; // Stop mode
  }
  else if (speed < BRAKE_THR) { // If speed is below braking threshold
    return -1; // Reverse mode (regenerative braking)
  }
  else { // If speed is above braking threshold
    return 1; // Forward mode (acceleration)
  }
}

//Source: Conversation with Bing, 4/26/2023
//(1) BLDC Motor Control With Arduino, Salvaged HD Motor, and Hall Sensors .... https://www.instructables.com/BLDC-Motor-Control-with-Arduino-salvaged-HD-motor/.
//(2) 3-Phase Brushless DC Motor Control with Hall Sensors. https://www.allaboutcircuits.com/industry-articles/3-phase-brushless-dc-motor-control-with-hall-sensors/.
//(3) AN bldc sensor - NXP. https://www.nxp.com/docs/en/application-note/AN1916.pdf.

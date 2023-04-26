
// Define the pins for the hall sensors
#define HALL_A 2
#define HALL_B 3
#define HALL_C 4

// Define the pins for the PWM signals
#define PWM_A 5
#define PWM_B 6
#define PWM_C 9

// Define the pins for the enable signals
#define EN_A_P 7
#define EN_A_N 8
#define EN_B_P 10
#define EN_B_N 11
#define EN_C_P 12
#define EN_C_N 13

// Define the pin for the current sense
#define CURR_SENSE A0

// Define the maximum PWM duty cycle (0-255)
#define MAX_PWM 200

// Define the frequency of the PWM signals (Hz)
#define PWM_FREQ 20000

// Define the number of samples for measuring current
#define NUM_SAMPLES 10

// Define the motor parameters (Ohm and Henry)
#define R_MOTOR 0.1
#define L_MOTOR 0.001

// Define the ADC reference voltage (V)
#define V_REF 5.0

// Define the current sense resistor value (Ohm)
#define R_SENSE 0.1

// Define the commutation table
const byte commTable[8][6] = {
  {0,0,0,0,0,0}, // no hall state
  {1,0,1,0,0,0}, // hall state 001
  {0,1,1,0,0,0}, // hall state 010
  {1,1,0,0,0,0}, // hall state 011
  {0,1,0,1,0,0}, // hall state 100
  {0,0,0,1,1,0}, // hall state 101
  {1,0,0,1,0,1}, // hall state 110
  {0,0,1,0,1,1} // hall state 111
};

// Declare global variables
byte hallState; // current hall state
byte pwmValue; // current PWM duty cycle
float iMotor; // current motor current (A)
float zMotor; // current motor impedance (Ohm)
float fMotor; // current motor frequency (Hz)

void setup() {
  
  // Set the hall sensor pins as inputs with pull-up resistors
  pinMode(HALL_A, INPUT_PULLUP);
  pinMode(HALL_B, INPUT_PULLUP);
  pinMode(HALL_C, INPUT_PULLUP);

  // Set the PWM pins as outputs and initialize them to low
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(PWM_C, OUTPUT);
  
  digitalWrite(PWM_A, LOW);
  digitalWrite(PWM_B, LOW);
  digitalWrite(PWM_C, LOW);

  // Set the enable pins as outputs and initialize them to low
  pinMode(EN_A_P, OUTPUT);
  pinMode(EN_A_N, OUTPUT);
  pinMode(EN_B_P, OUTPUT);
  pinMode(EN_B_N, OUTPUT);
  pinMode(EN_C_P, OUTPUT);
  pinMode(EN_C_N, OUTPUT);

  
  digitalWrite(EN_A_P, LOW);
  digitalWrite(EN_A_N, LOW);
  
digitalWrite(EN_B_P,
LOW);digitalWrite(EN_B_N,
LOW);digitalWrite(EN_C_P,
LOW);digitalWrite(EN_C_N,
LOW);

// Set the current sense pin as input
pinMode(CURR_SENSE,
INPUT);

// Set the PWM frequency and resolution
analogWriteFrequency(PWM_A,
PWM_FREQ);analogWriteFrequency(PWM_B,
PWM_FREQ);analogWriteFrequency(PWM_C,
PWM_FREQ);analogWriteResolution(8);

// Initialize the PWM duty cycle to zero
pwmValue = 
0;

// Initialize the motor frequency to zero
fMotor = 
0;

// Measure the motor impedance at startup by increasing the PWM duty cycle and measuring the current

while (pwmValue < MAX_PWM) {
  
    // Increase the PWM duty cycle by one step
    
    pwmValue++;
    
    // Apply the PWM duty cycle to one phase and enable it
    
    analogWrite(PWM_A,
    pwmValue);digitalWrite(EN_A_P,
    HIGH);digitalWrite(EN_A_N,
    LOW);

    // Wait for one PWM period to let the current stabilize
    
    delayMicroseconds(1000000 / PWM_FREQ);

    // Measure the average current by sampling the ADC
    
    iMotor = 
    measureCurrent();

    // Calculate the motor impedance by Ohm's law
    zMotor = V_REF * pwmValue / 255.0 / iMotor;

    // Check if the motor impedance is equal to the motor resistance
    // This means that the PWM frequency is equal to the low-pass transition frequency of the motor
    if (zMotor == R_MOTOR) {
      
      // Calculate the motor frequency by the PWM frequency and duty cycle
      fMotor = PWM_FREQ * pwmValue / 255.0;

      // Calculate the motor inductance by the impedance and frequency
      L_MOTOR = zMotor / (2 * PI * fMotor);

      // Break the loop
      break;
    }
  }

  // Disable all phases and set PWM duty cycle to zero
  digitalWrite(EN_A_P, LOW);
  digitalWrite(EN_A_N, LOW);
  digitalWrite(EN_B_P, LOW);
  digitalWrite(EN_B_N, LOW);
  digitalWrite(EN_C_P, LOW);
  digitalWrite(EN_C_N, LOW);

  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  analogWrite(PWM_C, 0);

  // Print the motor parameters to the serial monitor
  Serial.begin(9600);
  Serial.print("R_MOTOR = ");
  Serial.print(R_MOTOR);
  Serial.println(" Ohm");
  Serial.print("L_MOTOR = ");
  Serial.print(L_MOTOR);
  Serial.println(" H");
  Serial.print("fMotor = ");
  Serial.print(fMotor);
  Serial.println(" Hz");

}

void loop() {
  
  // Read the hall state from the sensor pins
  hallState = digitalRead(HALL_A) + (digitalRead(HALL_B) << 1) + (digitalRead(HALL_C) << 2);

  // Apply the commutation table to the enable pins
  digitalWrite(EN_A_P, commTable[hallState][0]);
  digitalWrite(EN_A_N, commTable[hallState][1]);
  
digitalWrite(EN_B_P,
commTable[hallState][2]);digitalWrite(EN_B_N,
commTable[hallState][3]);digitalWrite(EN_C_P,
commTable[hallState][4]);digitalWrite(EN_C_N,
commTable[hallState][5]);

// Apply the PWM duty cycle to the PWM pins
analogWrite(PWM_A,
pwmValue);analogWrite(PWM_B,
pwmValue);analogWrite(PWM_C,
pwmValue);

// Measure the average current by sampling the ADC
iMotor = 
measureCurrent();

// Print the current to the serial monitor
Serial.print("iMotor = ");
Serial.print(iMotor);
Serial.println(" A");

// Adjust the PWM duty cycle to keep the current below the maximum allowed pulse width
// This is done by using a simple proportional controller
if (iMotor * zMotor > V_REF * MAX_PWM / 255.0) {
  
    // Decrease the PWM duty cycle by one step
    
    pwmValue--;
    
    // Limit the PWM duty cycle to zero
    
    pwmValue = max(pwmValue,
    0);
    
} else if (iMotor * zMotor < V_REF * (MAX_PWM - 1) / 255.0) {
  
    // Increase the PWM duty cycle by one step
    
    pwmValue++;
    
    // Limit the PWM duty cycle to MAX_PWM
    
    pwmValue = min(pwmValue,
    MAX_PWM);
    
}

// Wait for one millisecond
delay(
1);

}

// Function to measure the average current by sampling the ADC
float measureCurrent() {
  
    // Declare a variable to store the sum of samples
    
    int sum = 
    0;

    // Declare a variable to store the average current
    
    float avgCurrent = 
    0;

    // Loop for NUM_SAMPLES times
    
    for (int i = 
        0; i < NUM_SAMPLES; i++) {
      
        // Read the ADC value and add it to the sum
        
        sum += analogRead(CURR_SENSE);

        // Wait for one microsecond
        
        delayMicroseconds(
        1);
        
    }

    // Calculate the average current by dividing the sum by NUM_SAMPLES and multiplying by V_REF / R_SENSE / 1024
    
    avgCurrent = sum / (float)NUM_SAMPLES * V_REF / R_SENSE / 
1024;

// Return the average current

return avgCurrent;

}

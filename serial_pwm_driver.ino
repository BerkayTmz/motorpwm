// Motor Control Pins
#define MOTOR_1_F 3
#define MOTOR_1_R 9
#define MOTOR_2_F 10
#define MOTOR_2_R 11

// Lidar PWM
#define LIDAR 5

// Servo PWM
#define SERVO 6


void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_1_F, OUTPUT);
  pinMode(MOTOR_1_R, OUTPUT);
  pinMode(MOTOR_2_F, OUTPUT);
  pinMode(MOTOR_2_R, OUTPUT);
}


void loop() {

// When there is serial data available
  while (Serial.available() > 0) {

    // Capture the key
    int header = Serial.read();
    delayMicroseconds(500);
    int value = Serial.read();

    // Classify according to key
    if (header == 10) {
      analogWrite(MOTOR_1_R, 0);
      analogWrite(MOTOR_1_F, value);
      Serial.write(value);// Actuate the value
    }
    else if (header == 15) {
      analogWrite(MOTOR_1_F, 0);
      analogWrite(MOTOR_1_R, value); // Actuate the value
    }
    else if (header == 20) {
      analogWrite(MOTOR_2_R, 0);
      analogWrite(MOTOR_2_F, value); // Actuate the value
    }
    else if (header == 25) {
      analogWrite(MOTOR_2_F, 0);
      analogWrite(MOTOR_2_R, value); // Actuate the value
    }
    else if (header == 30) {
      analogWrite(LIDAR, value); // Actuate the value
    }
    else if (header == 40) {
      analogWrite(SERVO, value); // Actuate the value
    }
    
  }

}

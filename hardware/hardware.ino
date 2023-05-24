#include <Servo.h>
#include <Firmata.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

//MIRS Motor Configuration
#define SERVOMIN 150   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600      // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400     // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates
#define SERVO_OSCILATOR_FREQ 27000000
#define PULSE_PIN 2
#define DIR_PIN 3
#define MICRO_STEPS 2
#define MIN_ANGLE 1.8  // in degrees


//M1
// unsigned long M1_rated_speed = ((0.17*1000L)/60); // At 6V
unsigned long M1_rated_speed = ((0.15 * 1000L) / 60);  // At 7.4V
//unsigned long M1_rated_speed = ((0.13*1000L)/60); // At 8.4V

//M2
unsigned long M2_rated_speed = ((0.13 * 1000L) / 60);  // At 7.2V
//unsigned long M2_rated_speed = ((0.1*1000L)/60); // At 8.4V

//M3
//unsigned long M3_rated_speed = ((0.185*1000L)/60); // At 6V
unsigned long M3_rated_speed = ((0.151 * 1000L) / 60);  // At 7.4V


//M4
//unsigned long M4_rated_speed = ((0.18*1000L)/60); // At 4.8V
//unsigned long M4_rated_speed = ((0.16*1000L)/60); // At 6V
unsigned long M4_rated_speed = ((0.14 * 1000L) / 60);  // At 7.2V

//M5
//unsigned long M5_rated_speed = ((0.18*1000L)/60); // At 4.8V
//unsigned long M5_rated_speed = ((0.16*1000L)/60); // At 6V
unsigned long M5_rated_speed = ((0.14 * 1000L) / 60);  // At 7.2V


/* MIRS COMMANDS*/
static const int MIRS_COMMAND = 0x04;
static const int COMMAND_MOTOR_GET_POSITION = 0x05;
static const int COMMAND_MOTOR_SET_POSITION = 0x06;
static const int COMMAND_MOTOR_SET_VELOCITY = 0x07;
static const int MOTOR_FEEDBACK_START_PIN = A0;
static const int MOTOR_FEEDBACK_END_PIN = A3;
static const int MOTORS = 1;
static const int SERVOS = MOTORS - 1;
static const int SERVO_PIN_START = 5;
static const int FEEDBACK_SIZE = MOTORS + 1;
static const int STEPS_PER_REV = 360 / MIN_ANGLE;
static const int PULSE_PER_REV = STEPS_PER_REV * MICRO_STEPS;

/* timer variables */
unsigned long currentMillis;  // store the current value from millis()
unsigned long previousMillis;
unsigned int samplingInterval = 19;  // how often to run the main loop (in ms)

byte MOTOR_RATED_VELOCITY[] = {
  M1_rated_speed,
  M2_rated_speed,
  M3_rated_speed,
  M4_rated_speed,
  M5_rated_speed
};

byte MOTOR_POSITION[MOTORS] = { 0 };
byte MOTOR_PRE_POSITION[MOTORS] = { 0 };
byte MOTOR_VELOCITY[MOTORS] = { 0 };  //in ms/deg
uint8_t FEEDBACK_DATA[FEEDBACK_SIZE] = { 0 };
Servo servos[SERVOS];


int motor_val;
int i = 0;
byte test[1] = { 0 };
int pos = 0;
int increment_angle = 60;
int total_angle = increment_angle;
unsigned long rated_speed = M2_rated_speed;         // ms/deg
unsigned long required_speed = ((1000L * 1) / 10);  // ms/deg
unsigned long time_delay;
unsigned long step_delay;
int steps_left;
int pulse_required;
int pulse_rate;


//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void set_servo(uint8_t servo_num, int angle) {
  Firmata.sendString("Servo starting...");

  increment_angle = abs(MOTOR_PRE_POSITION[servo_num + 1] - angle);
  time_delay = abs((increment_angle / (increment_angle - 1)) * (MOTOR_VELOCITY[servo_num + 1] - MOTOR_RATED_VELOCITY[servo_num]));  //ms

  // Serial.println("Total angle :"+String(total_angle));
  // Serial.println("Pre pos :"+String(pre_pos)+" Cur pos:"+String(total_angle));

  if (MOTOR_PRE_POSITION[servo_num + 1] < angle) {
    for (pos = MOTOR_PRE_POSITION[servo_num + 1] + 1; pos <= angle; pos += 1) {
      // in steps of 1 degree
      servos[servo_num].write(pos);
      // Serial.println(pos);
      delay(time_delay);
    }
    MOTOR_PRE_POSITION[servo_num + 1] = pos - 1;
  } else {
    for (pos = MOTOR_PRE_POSITION[servo_num + 1] - 1; pos >= angle; pos -= 1) {
      servos[servo_num].write(pos);
      // Serial.println(pos);
      delay(time_delay);
    }
    MOTOR_PRE_POSITION[servo_num + 1] = pos + 1;
  }
  delay(100);
  Firmata.sendString("Servo ran.");

  // int direction = 1;
  // float pre_state = MOTOR_STATE[servo_num];

  // pul_s = map(pre_state, 0, 180, SERVOMIN, SERVOMAX);
  // pul_e = map(new_state, 0, 180, SERVOMIN, SERVOMAX);

  // if (new_state < 0) {
  //   // Clock wise rotation
  //   for (pul = pul_s; pul >= pul_e; pul -= 1) {
  //     pwm.setPWM(servo_num, 0, pul);
  //     delay(velocity);
  //   }
  // } else {
  //   for (pul = pul_s; pul <= pul_e; pul += 1) {
  //     pwm.setPWM(servo_num, 0, pul);
  //     delay(velocity);
  //   }
  // }
}

// Set stepper angle
void set_stepper(float angle) {
  Firmata.sendString("Stepper starting...");
  if (angle > 0) {
    digitalWrite(DIR_PIN, HIGH);
  } else {
    digitalWrite(DIR_PIN, LOW);
  }

  pulse_required = (angle / 360) * PULSE_PER_REV;            // Total pulses required
  pulse_rate = abs(360*PULSE_PER_REV/MOTOR_VELOCITY[0]);  // pulse/sec
  steps_left = abs(pulse_required);

  step_delay = (1000L * 1000L) / (2 * (pulse_rate));  // micro sec / pulse

  // decrement the number of steps, moving one step each time:
  while (steps_left > 0) {
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(step_delay);
    digitalWrite(PULSE_PIN, LOW);
    delayMicroseconds(step_delay);
    steps_left--;
  }
  MOTOR_POSITION[0] = (byte)(int)abs(angle);
  delay(100);
  Firmata.sendString("Stepper ran.");
}

void set_motor_speed(byte data_size, byte *data) {
  if (data_size >= (2 * MOTORS)) {
    // Parse out the motor values (with 8 elements)
    for (i = 0; i < MOTORS * 2; i += 2) {
      // Decode bits & combine to bits
      motor_val = (int)((data[i + 1] & 0x7F) << 7) | (data[i] & 0x7F);
      MOTOR_VELOCITY[i / 2] = motor_val;
    }
  } else {
    Firmata.sendString("Canot set motor velocity! Size is less.");
  }
}

void set_motor_position(byte data_size, byte *data) {
  // Expect motor values with 2 bytes (having 7 bits data in each byte)
  if (data_size >= (2 * MOTORS)) {
    // Parse out the motor values (with 8 elements)
    for (i = 0; i < MOTORS * 2; i += 2) {
      // Decode bits & combine to bits
      motor_val = (int)((data[i + 1] & 0x7F) << 7) | (data[i] & 0x7F);
      // test[0] = motor_val;
      // Firmata.sendSysex(MIRS_COMMAND, 2, test);
      if (i == 0) {
        Firmata.sendString("Setting stepper");
        set_stepper(motor_val);
      } else {
        Firmata.sendString("Setting servo");
        set_servo(i / 2, motor_val);
      }
    }
  } else {
    Firmata.sendString("Size is less!");
  }
}

void send_motor_feedback() {
  // Add feedback command
  FEEDBACK_DATA[0] = COMMAND_MOTOR_GET_POSITION;
  FEEDBACK_DATA[1] = MOTOR_POSITION[0];  //Assuming accurate


  //Sending only servo feedback positions
  for (i = 0; i < MOTORS - 1; i += 1) {
    FEEDBACK_DATA[i + 2] = analogRead(MOTOR_FEEDBACK_START_PIN + i);
  }

  // Send the feedback
  Firmata.sendSysex(MIRS_COMMAND, FEEDBACK_SIZE, FEEDBACK_DATA);
}

void send_error(byte data) {
  Firmata.sendString(data);
}

void mirs_command_handler(byte command, byte argc, byte *argv) {
  switch (command) {
    case COMMAND_MOTOR_SET_POSITION:
      // Set motor
      set_motor_position(argc, argv);
      break;
    case COMMAND_MOTOR_GET_POSITION:
      // Send feedback
      send_motor_feedback();
      break;
    case COMMAND_MOTOR_SET_VELOCITY:
      set_motor_speed(argc, argv);
      break;
  }
}

void sysexCallback(byte command, byte argc, byte *argv) {
  switch (command) {
    case MIRS_COMMAND:
      if (argc < 1) return;
      // Extract top level command and update argv, argc and process the rest
      mirs_command_handler(argv[0], argc - 1, argv + 1);
      break;
  }
}

void setup() {
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  for (i = 0; i < SERVOS; i++) {
    servos[i] = Servo();
    servos[i].attach(i + SERVO_PIN_START);
  }
  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);
  Firmata.attach(START_SYSEX, sysexCallback);

  Firmata.begin(57600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for ATmega32u4-based boards and Arduino 101
  }
}

void loop() {
  while (Firmata.available())
    Firmata.processInput();
}

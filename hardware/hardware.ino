#include <Wire.h>
#include <Firmata.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Motor Configuration
#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600     // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_OSCILATOR_FREQ 27000000
#define PULSE_PIN 2
#define DIR_PIN 3
#define ENA_PIN 4
#define MIN_ANGLE 1.8     // in degrees
#define PULSE_PER_REV 200 // change this to fit the number of steps per revolution
#define MOTOR_FEEDBACK_START_PIN 1
#define MOTOR_FEEDBACK_END_PIN 6

// Robot state and other variables
float MOTOR_STATE[6] = {0};
uint8_t servonum = 0;
float servo_angle;
double pul_s;
double pul_e;
double pul;
float pre_state;
uint8_t pin = 0;
float motor_state;

/* timer variables */
unsigned long current_milis;  // store the current value from millis()
unsigned long previous_millis; // for comparison with current_millis
int sampling_interval = 19; // how often to run the main loop (in ms)

// Set servo angle
void set_servo(uint8_t servo_num, float new_state, float velocity)
{
    int direction = 1;
    float pre_state = STATE[servo_num];

    pul_s = map(pre_state, 0, 180, SERVOMIN, SERVOMAX);
    pul_e = map(new_state, 0, 180, SERVOMIN, SERVOMAX);

    if (new_state < 0)
    {
        // Clock wise rotation
        for (pul = pul_s; pos >= pul_e; pos -= 1)
        {
            pwm.setPWM(servo_num, pul);
            delay(velocity);
        }
    }
    else
    {
        for (pul = pul_s; pos <= pul_e; pos += 1)
        {
            pwm.setPWM(servo_num, pul);
            delay(velocity);
        }
    }
}

// Set stepper angle
void set_stepper(float angle, float rpm)
{

    digitalWrite(dir_pin, HIGH);

    int pulse_required = (angle / 360) * pulse_per_rev; // Total pulses required
    int pulse_rate = (pulse_per_rev * rpm / 60)         // pulse/sec

        int steps_left = abs(pulse_required);

    unsigned long last_step_time = 0;
    unsigned long step_delay = 1000L * 1000L / pulse_rate; // micro sec / pulse

    // decrement the number of steps, moving one step each time:
    while (steps_left > 0)
    {
        unsigned long now = micros();
        // move only if the appropriate delay has passed:
        if (now - last_step_time >= step_delay)
        {
            // get the timeStamp of when you stepped:
            last_step_time = now;

            // step the motor
            digitalWrite(pulse_pin, HIGH);

            // decrement the steps left:
            steps_left--;
        }
        else
        {
            digitalWrite(pulse_pin, LOW);
        }
    }
}

void set_motor_state(int motor_num,float motor_angle,float speed){
    if(motor_num==1){
        set_stepper(motor_angle, speed);
    }else{
        set_stepper(motor_num, motor_angle, speed);
    }
}

void sysexCallback(byte command, byte argc, byte *argv)
{

    Serial.print("command :");
    Serial.println(command);

    Serial.print("argc :");
    Serial.println(argc);

    switch (command)
    {
    case "MOTOR_VALS":
        /* Set motor values */
        set_motor_state(argc, argv[0], argv[1]);
        break;
    case "GET_FEEBACK":
        /* Send motor feedback */
        Firmata.sendSysex(command, 1, MOTOR_STATE);
        break;
     default:
        break;
    }
}

void setup()
{
    Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);

    // Attach callbacks to commands
    Firmata.attach(START_SYSEX, sysexCallback);
    Firmata.begin(57600);
}

void loop()
{
    current_milis = millis();
    if (current_milis - previous_millis > sampling_interval)
    {

        // Continuosly read data and set feedback
        previous_millis += sampling_interval;
        while (Firmata.available())
        {
                Firmata.processInput();
        }
        for (pin = MOTOR_FEEDBACK_START_PIN; pin < MOTOR_FEEDBACK_END_PIN; pin++)
        {
            motor_state = analogRead(pin);
            if (motor_state != MOTOR_STATE[pin])
            {
                MOTOR_STATE[pin] = motor_state;
            }
        }
    }
}

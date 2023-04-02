const int pulse_pin = 2;
const int dir_pin = 3;
const int ena_pin = 4;
const float min_angle = 1.8; // in degrees
float angle = 0.0;
const int pulse_per_rev = 200; // change this to fit the number of steps per revolution

void setup() {
  pinMode(pulse_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(ena_pin, OUTPUT);
  digitalWrite(ena_pin, LOW);
  Serial.begin(9600);
}

void step(float angle,float rpm){
  digitalWrite(dir_pin, HIGH);

  int pulse_required = (angle/360)*pulse_per_rev;   // Total pulses required
  int pulse_rate = (pulse_per_rev*rpm/60) // pulse/sec 
  
  int steps_left = abs(pulse_required);
  
  unsigned long last_step_time = 0;
  unsigned long step_delay = 1000L * 1000L / pulse_rate; //micro sec / pulse

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
      digitalWrite(pulse_pin,HIGH);

      // decrement the steps left:
      steps_left--;
  }else{
    digitalWrite(pulse_pin,LOW);
  }
}
  }
void loop() {
  int speed_rpm = 60; //in rpm
  if (Serial.available()>0) {
    int angle = Serial.read();  //in degrees
    Serial.print("Angle :");
    Serial.println(angle);
    step(angle,speed_rpm);
  }
}
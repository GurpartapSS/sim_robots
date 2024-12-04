#include <PID_v1.h>

#define enR 13
#define in1_r 12
#define in2_r 14
#define encoder_phaseA_r 32 //green
#define encoder_phaseB_r 33 //yellow

#define enL 15
#define in1_l 0
#define in2_l 2
#define encoder_phaseA_l 18 //green
#define encoder_phaseB_l 19 //yellow

// L298N H-Bridge Connection PINs
#define L298N_enA 13  // PWM
#define L298N_enB 14  // PWM
#define L298N_in4 23  // Dir Motor B
#define L298N_in3 22  // Dir Motor B
#define L298N_in2 27  // Dir Motor A
#define L298N_in1 12  // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 32  // Interrupt 
#define right_encoder_phaseB 33  
#define left_encoder_phaseA 18   // Interrupt
#define left_encoder_phaseB 19


// double cmd = 0.0;
unsigned int encoder_counter_r= 0;
String right_encoder_dir = "p";
double wheel_vel_right = 0.0; // RPM = encoder_pulse *(60 / (11*35) )
unsigned int encoder_counter_l= 0;
String left_encoder_dir = "p";
double wheel_vel_left = 0.0; // RPM = encoder_pulse *(60 / (11*35) )
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
char value[] = "00.00";
uint8_t value_idx = 0; 
bool is_cmd_complete = false;
bool is_right_wheel_frwd = true;
bool is_left_wheel_frwd = true;
double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;

unsigned long last_millis = 0;
const unsigned long interval = 100;

double right_wheel_cmd = 0.0;
double left_wheel_cmd = 0.0;

double kp_r = 11.5;
double ki_r = 7.5;
double kd_r = 0.1;
double kp_l = 12.8;
double ki_l = 8.3;
double kd_l = 0.1;
PID rightMotor(&wheel_vel_right, &right_wheel_cmd, &right_wheel_cmd_vel, kp_r, ki_r, kd_r, DIRECT);
PID leftMotor(&wheel_vel_left, &left_wheel_cmd, &left_wheel_cmd_vel, kp_l, ki_l, kd_l, DIRECT);

void setup() {
  
  pinMode(enR, OUTPUT);
  pinMode(in1_r, OUTPUT);
  pinMode(in2_r, OUTPUT);
  pinMode(encoder_phaseA_r, INPUT);
  pinMode(encoder_phaseB_r, INPUT);

  pinMode(enL, OUTPUT);
  pinMode(in1_l, OUTPUT);
  pinMode(in2_l, OUTPUT);
  pinMode(encoder_phaseA_l, INPUT);
  pinMode(encoder_phaseB_l, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoder_phaseA_r), encodercallbackR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_phaseA_l), encodercallbackL, RISING);

  digitalWrite(in1_r, HIGH);
  digitalWrite(in2_r, LOW);
    digitalWrite(in1_l, HIGH);
  digitalWrite(in2_l, LOW);

  Serial.begin(115200);
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
}

void loop() {
  if(Serial.available()) {
    char cmd = Serial.read();
    if(cmd == 'r') {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
    } else if(cmd == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    } else if(cmd == 'p') {
      if(is_right_wheel_cmd && !is_right_wheel_frwd){
        digitalWrite(in1_r, HIGH - digitalRead(in1_r));
        digitalWrite(in2_r, HIGH - digitalRead(in2_r));
        is_right_wheel_frwd = true;
      } else if(is_left_wheel_cmd && !is_left_wheel_frwd){
        digitalWrite(in1_l, HIGH - digitalRead(in1_l));
        digitalWrite(in2_l, HIGH - digitalRead(in2_l));
        is_left_wheel_frwd = true;
      }
    }else if(cmd == 'n') {
      if(is_right_wheel_cmd && is_right_wheel_frwd){
        digitalWrite(in1_r, HIGH - digitalRead(in1_r));
        digitalWrite(in2_r, HIGH - digitalRead(in2_r));
        is_right_wheel_frwd = false;
      } else if(is_left_wheel_cmd && is_left_wheel_frwd){
        digitalWrite(in1_l, HIGH - digitalRead(in1_l));
        digitalWrite(in2_l, HIGH - digitalRead(in2_l));
        is_left_wheel_frwd = false;
      }
    } else if (cmd == ',') {
      if(is_right_wheel_cmd) {
        right_wheel_cmd_vel = atof(value);
      } else if(is_left_wheel_cmd) {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    } else {
      if(value_idx < 5) {
        value[value_idx] = cmd;
        value_idx++;
      }
    }
  }
  // analogWrite(enR, 200);
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval) {
    wheel_vel_right = 10 * (encoder_counter_r * 60.0 / 385.0) * 0.10472;
    wheel_vel_left = 10 * (encoder_counter_l * 60.0 / 385.0) * 0.10472;

    rightMotor.Compute();
    leftMotor.Compute();
    analogWrite(enR, right_wheel_cmd);
    analogWrite(enR, left_wheel_cmd);
    if(right_wheel_cmd_vel == 0.0) {
      right_wheel_cmd = 0.0;
    }
    if(left_wheel_cmd_vel == 0.0) {
      left_wheel_cmd = 0.0;
    }
    String encoder_read = "r" + right_encoder_dir + String(wheel_vel_right) + "l" + left_encoder_dir + String(wheel_vel_left) + ',';
    Serial.println(encoder_read);
    last_millis = current_millis;
    encoder_counter_r = 0;
    encoder_counter_l = 0;
    delay(100);
  }

}

void encodercallbackR() {
  encoder_counter_r++;
  if(digitalRead(encoder_phaseB_r) == HIGH){
    right_encoder_dir = "p";
  } else {
    right_encoder_dir = "n";
  } 
}
void encodercallbackL() {
  encoder_counter_l++;
  if(digitalRead(encoder_phaseB_l) == HIGH){
    left_encoder_dir = "p";
  } else {
    left_encoder_dir = "n";
  } 
}

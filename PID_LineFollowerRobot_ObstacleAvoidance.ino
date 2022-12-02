#include <Servo.h>
Servo head;

#define SERVO_PIN     9  

//Define L298N Dual H-Bridge Motor Controller Pins
#define RightDirectPin1  12    //Right Motor direction pin 1 
#define RightDirectPin2  11    //Right Motor direction pin 1
#define speedPinL 6    //Left PWM pin 
#define LeftDirectPin1  7    //Left Motor direction pin 1
#define LeftDirectPin2  8   //Left Motor direction pin 1 
#define speedPinR 3    // Right PWM pin 

#define Echo_PIN 2
#define Trig_PIN 10

#define LFSensor_0 A0  
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  

int sensor[5]={0, 0, 0, 0, 0};

float Kp = 37, Ki = 0.1, Kd = 0.2;
float error = 0, e_dot = 0, e_old = 0, E = 0, u = 0;
int initial_motor_speed = 100;
int PWA_left, PWA_right;
int oldtime = 0;
#define SPEED 120 
bool flag = 0;
int distance;
 
void go_AdvancePID(){
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  analogWrite(speedPinL, PWA_left);
  analogWrite(speedPinR, PWA_right);
}
void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(SPEED,SPEED);
}
void go_Left()  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  set_Motorspeed(SPEED,SPEED);
}
void go_Right()  //Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(SPEED,SPEED);
}
void go_Back()  //Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  set_Motorspeed(SPEED,SPEED);
}
void stop_Stop()   
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
  set_Motorspeed(SPEED,SPEED);
}
void set_Motorspeed(int SPEED_L, int SPEED_R){
  analogWrite(speedPinL,SPEED);
  analogWrite(speedPinR,SPEED);
}

void setup()
{ 
  head.attach(SERVO_PIN);
  head.write(90);
  delay(100);
  
  pinMode(RightDirectPin1, OUTPUT); 
  pinMode(RightDirectPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();

  pinMode(Trig_PIN, OUTPUT); 
  pinMode(Echo_PIN,INPUT); 

  pinMode(LFSensor_0,INPUT); 
  pinMode(LFSensor_1,INPUT); 
  pinMode(LFSensor_2,INPUT); 
  pinMode(LFSensor_3,INPUT); 
  pinMode(LFSensor_4,INPUT); 
  
  Serial.begin(9600);    
} 

void loop(){ 
  distance = watch();
  if(distance > 25){
    read_sensor_values();
    calculate_PID();
    auto_tracking();
    go_AdvancePID();
  }
  distance = watch();
  if(distance < 20){
    auto_avoidance();
  }
  Serial.println(distance);
}

/*detection of ultrasonic distance*/
int watch(){
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(2);                                                                              
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_PIN,LOW);
  long echo_distance1=pulseIn(Echo_PIN,HIGH);
  echo_distance1=echo_distance1*0.01657; //how far away is the object in cm
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(2);                                                                              
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_PIN,LOW);
  long echo_distance2=pulseIn(Echo_PIN,HIGH);
  echo_distance2=echo_distance2*0.01657; //how far away is the object in cm
 
  long echo_distance = (echo_distance1+echo_distance2)/2;
  return round(echo_distance);
}

void auto_avoidance(){
  go_Right();
  set_Motorspeed(160,160);
  delay(740);
  go_Advance();
  set_Motorspeed(200,200);
  delay(500);
  go_Left();
  set_Motorspeed(160,160);
  delay(740);
  go_Advance();
  set_Motorspeed(200,200);
  delay(800);
  go_Left();
  set_Motorspeed(160,160);
  delay(740);
  go_Advance();
  set_Motorspeed(200,200);
  delay(500);
  go_Right();
  set_Motorspeed(160,160);
  delay(740);
}

void read_sensor_values(){
  sensor[0] = !digitalRead(LFSensor_0);
  sensor[1] = !digitalRead(LFSensor_1);
  sensor[2] = !digitalRead(LFSensor_2);
  sensor[3] = !digitalRead(LFSensor_3);
  sensor[4] = !digitalRead(LFSensor_4);

  if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)){
    error = 4;
  }
  
  else if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)){
    error = 3;
  }
  
  else if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0)){
    error = 2;
  }
  
  else if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)){
    error = 1;
  }

  else if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)){
    error = 0;
  }

  else if((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)){
    error = -1;
  }

  else if((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)){
    error = -2;
  }
  
  else if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)){
    error = -3;
  }

  else if((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)){
    error = -4;
  }
}

void calculate_PID(){
  e_dot = error - e_old;
  E = E + error;
  u = Kp*error + Ki*E + Kd*e_dot;
  e_old = error;
}

void auto_tracking(){
  int left_motor_speed = initial_motor_speed + u;
  int right_motor_speed = initial_motor_speed - u;

  PWA_left = constrain(left_motor_speed, 0, 255);
  PWA_right = constrain(right_motor_speed, 0, 255);

  if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)){
    PWA_left = 0;
    PWA_right = 0;
  }
  
  if(millis()-oldtime > 5000){
    oldtime = millis();
    if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)){
    PWA_left = 0;
    PWA_right = 0;
    }
  }
}

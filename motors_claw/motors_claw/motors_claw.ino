#include <Adafruit_NeoPixel.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <Servo.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8MultiArray.h>

std_msgs::Int8 miss_stat;
Servo leftMotor;
Servo rightMotor;
uint32_t c;
int leftIR = A0;
int rightIR = A1;
int back_leftIR = A2;
int back_rightIR = A4;
const int clawIR = A0;
int eStopPin = A8;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int constant = 0;
// the data pin for the NeoPixels
int headlight1 = 9;
int headlight2 = 10;
int backlight1 = 8;
int backlight2 = 11;
// How many NeoPixels we will be using, change accordingly
int numPixels = 144;
bool turnLeft = false;
bool turnRight = false;
bool straight = true;
char light_state = 'f';
bool use_cmd = true;
bool can_picked = false;

unsigned long previousMillis = 0;        // will store last time LED was updated 
unsigned long interval = 60;           // interval at which to blink (milliseconds)
uint16_t i = 8;
uint16_t j = 8;
uint16_t k = 0;

//claw and arm 
#define ARM_PIN 2
#define CLAW_PIN 3
Servo claw;  // create servo object to control a servo
Servo arm;
// twelve servo objects can be created on most boards
int pos_claw = 0;    // variable to store the servo position
int pos_arm = 0;

boolean  eStopTriggered;


ros::NodeHandle  nh;

std_msgs::Bool e_stop_msg;
std_msgs::Bool can_picked_msg;


int velocity = 0;
int new_velocity=0;
int angular = 0;
double angular_coeff = 0;
int angular_div = 100; //what the angular component needs to be divided by

// Instatiate the NeoPixel from the library
Adafruit_NeoPixel h1 = Adafruit_NeoPixel(numPixels, headlight1, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel h2 = Adafruit_NeoPixel(numPixels, headlight2, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel b1 = Adafruit_NeoPixel(numPixels, backlight1, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel b2 = Adafruit_NeoPixel(numPixels, backlight2, NEO_GRBW + NEO_KHZ800);


int irRead() {
  int averaging = 0;             //  Holds value to average readings
  int distance = 0;

  // Get a sampling of 5 readings from sensor
  for (int i=0; i<5; i++) {
    distance = analogRead(clawIR);
    averaging = averaging + distance;
    delay(55);      // Wait 55 ms between each read
                    // According to datasheet time between each read
                    //  is -38ms +/- 10ms. Waiting 55 ms assures each
                    //  read is from a different sample
  }
  distance = averaging / 5;      // Average out readings
  return(distance);              // Return value
}

// claw and arm functions 
void arm_up(){
  arm.write(169);
  delay(50);
}

void arm_down(){
  arm.write(10);
  delay(50);
}

void open_claw(){
 claw.write(180);              // tell servo to go to position in variable 'pos'
 delay(50);
}

void close_claw(){
  claw.write(0);
  delay(50);
}

void position_forward(){
  while (irRead() < 450) { //IR distance for can
  //scoots forward after claw is down and open, to position can correctly for pickup, speeds and times need to be calibrated
  leftMotorSpeed = 78; //write slow speed to left and right motors
  rightMotorSpeed = 78;
  leftMotor.write(leftMotorSpeed);
  rightMotor.write(rightMotorSpeed); 
  
  }
  leftMotorSpeed = 85; // write stopped speed to left and right motors.
  rightMotorSpeed = 85;
  leftMotor.write(85);
  rightMotor.write(85); 
}


void claw_cycle(){
  use_cmd = false;
  leftMotor.write(85);
  rightMotor.write(85); 
  open_claw();
  delay(500);
  nh.spinOnce();
  arm_down();
  delay(1000);
  nh.spinOnce();
  position_forward();
  close_claw();
  delay(1000);
  arm_up();
  delay(5000);
  use_cmd = true;
  can_picked = true;
  nh.spinOnce();
  
}

void claw_cb(const std_msgs::Int8& miss_stat){
  if (miss_stat.data == 3){
    claw_cycle();
  }
  if (miss_stat.data == 5){
    can_picked = false;
  }
}

void cb(const geometry_msgs::Twist& twist_msg){
  if (use_cmd == true){
    turnRight = false;
    turnLeft = false;
    straight = true;
  
	//convert msg (in linear and angular components) into left and right motor speeds
    velocity = (int) twist_msg.linear.x;
    leftMotorSpeed= velocity;
    rightMotorSpeed = velocity;
  
    angular = (int) twist_msg.angular.z;
  
    if (angular < 0){
      turnLeft = true;
      turnRight = false;
      straight = false;
    
      //negative angle = turn left
      leftMotorSpeed = velocity + angular;
      rightMotorSpeed = velocity;

      if (leftMotorSpeed>100){
        leftMotorSpeed == 100;
      }
      else if (leftMotorSpeed<-100){
        leftMotorSpeed == -100;
    }
    if (rightMotorSpeed>100){
      rightMotorSpeed == 100;
    }
    else if (rightMotorSpeed<-100){
      rightMotorSpeed == -100;
    }

  }
  if (angular > 0){
    turnRight = true;
    turnLeft = false;
    straight = false;
    
    //positive angle = turn right
    leftMotorSpeed = velocity;
    rightMotorSpeed = velocity - angular;

    if (leftMotorSpeed>100){
      leftMotorSpeed == 100;
    }
    else if (leftMotorSpeed<-100){
      leftMotorSpeed == -100;
    }
    if (rightMotorSpeed>100){
      rightMotorSpeed == 100;
    }
    else if (rightMotorSpeed<-100){
      rightMotorSpeed == -100;
    }
  }
  leftMotorSpeed = map(leftMotorSpeed,-100,100,140,30);
  rightMotorSpeed = map(rightMotorSpeed,-100,100,140,30);
  }
}


ros::Publisher e_stop("e_stop", &e_stop_msg);
ros::Publisher can_picked_pub("can_picked_pub", &can_picked_msg);

ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", &cb);
ros::Subscriber <std_msgs::Int8> sub2("miss_stat", &claw_cb);



void setup() {
  pinMode(eStopPin, INPUT);
  h1.begin();  // initialize the strip
  h2.begin();
  b1.begin();
  b2.begin();
  h1.clear();  // Initialize all pixels to 'off'
  h2.clear();
  b1.clear();
  b2.clear();
  leftMotor.attach(7);
  rightMotor.attach(6);
  arm.attach(ARM_PIN);
  claw.attach(CLAW_PIN);
  arm.write(169);

  claw.write(180);
  
  
  // set the colors for the strip
  for( int i = 0; i < numPixels; i++ ){
       h1.setPixelColor(i, 255, 255, 255);
       //h1.setBrightness(50);
       h2.setPixelColor(i, 255, 255, 255);
       //h2.setBrightness(50);
  }
  
  h1.show();
  h2.show();

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(e_stop);
  nh.advertise(can_picked_pub);
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.spinOnce();
}

void loop() {
  //read and calculate IR data
  int leftIR_range;
  int rightIR_range;
  int back_leftIR_range;
  int back_rightIR_range;
      
  leftIR_range = sharpRange(leftIR);
  rightIR_range = sharpRange(rightIR);
  back_leftIR_range = sharpRange(back_leftIR);
  back_rightIR_range = sharpRange(back_rightIR);
  
  eStopTriggered = readEstop();
  
  //if estop is on, set motor speeds to zero
  if (eStopTriggered==true){
    leftMotorSpeed = 85;
    rightMotorSpeed = 85;
  }

  //default_lights();
  
  if (turnLeft==true && turnRight==false && straight==false){
    turn_left_lights(b1.Color(255,0,0));
  }
  else if (turnRight==true && turnLeft==false && straight==false){
    turn_right_lights(b2.Color(255,0,0));
  }
  else if (turnRight==false && turnLeft==false && straight==true){
    straight_lights();
  }
  
  //if there's no ground set motor speeds to zero
  if (leftIR_range > 50.0 || rightIR_range > 50.0){
    //leftMotorSpeed = 85;
    //rightMotorSpeed = 85;
  }
  
  if (leftMotorSpeed < 0 || rightMotorSpeed < 0){
    if (back_leftIR_range < 25.0 || back_rightIR_range < 25.0){
      leftMotorSpeed = 85;
      rightMotorSpeed = 85;
    }
  }
  
  if (use_cmd == true){
  //write the motor speeds to the motor
  leftMotor.write(leftMotorSpeed);
  rightMotor.write(rightMotorSpeed); 
  }
  
  e_stop_msg.data = eStopTriggered;
  can_picked_msg.data = can_picked;
  
  //publish the data
  e_stop.publish( &e_stop_msg);
  can_picked_pub.publish( &can_picked_msg);
  
  nh.spinOnce();
}

float sharpRange (int sensornum) {
  int rawData = analogRead(sensornum);
  //Serial.println(rawData);
  float volts = rawData * 0.0048828125;
  float range = 65*pow(volts,-1.10);
  return range;
}

void turn_left_lights(uint32_t c){
  if ((unsigned long)(millis() - previousMillis) >= interval) {
    previousMillis = millis();
    b2.setPixelColor(i,b2.Color(255,0,0));
    b1.setPixelColor(i,b1.Color(0,0,255));
    b2.show();
    i--;
    if (i==0){
      b1.show();
      for(uint16_t i=0; i<b2.numPixels(); i++) {
        b2.setPixelColor(i  , 0); // Draw new pixel
      }
      b2.show();
      i=8;
    }
  }
}

void turn_right_lights(uint32_t c){
  if ((unsigned long)(millis() - previousMillis) >= interval) {
    previousMillis = millis();
    b1.setPixelColor(j,b1.Color(255,0,0));
    b2.setPixelColor(j,b2.Color(0,0,255));
    b1.show();
    j--;
    if (j==0){
      b2.show();
      for(uint16_t i=0; i<b1.numPixels(); i++) {
        b1.setPixelColor(i  , 0); // Draw new pixel
      }
      b1.show();
      j=8;
    }
  }
}

void straight_lights(){
  if ((unsigned long)(millis() - previousMillis) >= interval) {
    previousMillis = millis();
    b1.setPixelColor(k,b1.Color(0,255,0));
    b2.setPixelColor(k,b2.Color(0,255,0));
    k++;
    if (k==8){
      b1.show();
      b2.show();
      k=0;
    }
  }  
}

boolean readEstop(){
  int eStop = analogRead(eStopPin);
  if (eStop < 100){
    return true;}
  return false;
}

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

Servo leftMotor;
Servo rightMotor;
Servo lidar;
uint32_t c;
int leftIR = A0;
int rightIR = A1;
int back_leftIR = A2;
int back_rightIR = A3;
int eStopPin = A8;
int leftMotorSpeed = 85;
int rightMotorSpeed = 85;
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

unsigned long previousMillis = 0;        // will store last time LED was updated 
unsigned long interval = 60;           // interval at which to blink (milliseconds)
uint16_t i = 8;
uint16_t j = 8;
uint16_t k = 0;

boolean  eStopTriggered;

char* *ir_sense_string[4];

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

void cb(const geometry_msgs::Twist& twist_msg){
  turnRight = false;
  turnLeft = false;
  straight = true;
  
	//convert msg (in linear and angular components) into left and right motor speeds
  velocity = (int) twist_msg.linear.x;
  //velocity = map(velocity,-60,60,-100,100);
  leftMotorSpeed= velocity;
  rightMotorSpeed = velocity;
  
  angular = (int) twist_msg.angular.z;
  //angular = map(angular,-100,100,-1,1);
  
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

ros::NodeHandle  nh;

std_msgs::Bool e_stop_msg;

std_msgs::Int16MultiArray ir;
std_msgs::Int16MultiArray mcb;

int ir_array;
ros::Publisher e_stop("e_stop", &e_stop_msg);
ros::Publisher ir_sensors("ir_sensors", &ir);
ros::Publisher motorcb("motorcb", &mcb);

ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", &cb);


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
  lidar.attach(4);
  Servo claw;
  claw.attach(3);
  claw.write(180);
  
  // set the colors for the strip
  for( int i = 0; i < numPixels; i++ ){
       h1.setPixelColor(i, 255, 255, 255);
       //h1.setBrightness(50);
       h2.setPixelColor(i, 255, 255, 255);
       //h2.setBrightness(50);
  }
  
  //h1.show();
  //h2.show();

  nh.initNode();
  nh.advertise(e_stop);
  nh.advertise(ir_sensors);
  nh.advertise(motorcb);
  nh.subscribe(sub);
  
  
  //setup both arrays
  ir.data = (int16_t*) malloc(5);
  ir.data_length = 4;
  
  mcb.data = (int16_t*) malloc(3);
  mcb.data_length = 2;
  lidar.write(90);
}

void loop() {
  //read and calculate IR data
  float leftIR_range = 0;
  int rightIR_range = 0;
  int back_leftIR_range = 0;
  int back_rightIR_range = 0;
      
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
  //if (leftIR_range > 50.0 || rightIR_range > 50.0){
  //  leftMotorSpeed = 85;
  //  rightMotorSpeed = 85;
  //}
 
  //write the motor speeds to the motor
  leftMotor.write(leftMotorSpeed);
  rightMotor.write(rightMotorSpeed); 

  
  //set the data to be published
  ir.data[0] = leftIR_range;
  ir.data[1] = rightIR_range;
  ir.data[2] = back_leftIR_range;
  ir.data[3] = back_rightIR_range;

  mcb.data[0] = leftMotorSpeed;
  mcb.data[1] = rightMotorSpeed;
  
  e_stop_msg.data = eStopTriggered;
  
  //publish the data
  motorcb.publish( &mcb );
  ir_sensors.publish( &ir );
  e_stop.publish( &e_stop_msg);
  
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

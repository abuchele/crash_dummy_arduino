/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

Servo right_motor;  // create servo object to control a servo
Servo left_motor;  // create servo object to control a servo
unsigned long time;
std_msgs::Int8 miss_stat;

// twelve servo objects can be created on most boards

int right_speed = 0;    // variable to store the servo position
int left_speed = 0;    // variable to store the servo position

void cb(const std_msgs::Int8& miss_stat){
  spin_motor();
  if (miss_stat.data == 2){
    stop_motor();
  }
  delay(100000);
}

ros::Subscriber <std_msgs::Int8> sub2("img_rec/miss_stat", &cb);
ros::NodeHandle  nh;


void setup() {
  right_motor.attach(6);  // attaches the servo on pin 9 to the servo object
  left_motor.attach(7);  // attaches the servo on pin 9 to the servo object
  nh.initNode();
  nh.subscribe(sub2);

}


void spin_motor(){
  right_motor.write(80);              // tell servo to go to position in variable 'pos'
  left_motor.write(100);
}

void stop_motor(){
  right_motor.write(85);              // tell servo to go to position in variable 'pos'
  left_motor.write(85);
}

void loop() {
  nh.spinOnce();

}

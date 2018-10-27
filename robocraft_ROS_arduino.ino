#include <Encoder.h>
#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
//ROS messages
std_msgs::Bool Lin_front_msg;
std_msgs::Bool Lin_back_msg;
std_msgs::Float32 front_distance_msg;
std_msgs::Float32 right_distance_msg;
std_msgs::Float32 left_distance_msg;
std_msgs::UInt8MultiArray rgb_leds_msg;
geometry_msgs::Pose2D pose_msg;
geometry_msgs::Pose2D initial_pose_msg;
geometry_msgs::Twist cmd_vel_msg;

// Defining the pin configuration
#define PIN 10
#define NUMPIXELS 2
#define BRIGHTNESS 25
#define MotLeft_A 2
#define MotLeft_B 3
#define MotRight_A 18
#define MotRight_B 19
#define DirRight1 5
#define DirRight2 6
#define DirLeft1 7
#define DirLeft2 8
#define PWM1 4
#define PWM2 9
#define pi 3.1415
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Encoder encL(MotRight_A, MotRight_B);
Encoder encR(MotLeft_B, MotLeft_A);

// Defining threshold values for line sensor
long NL, NR;
int front_white = 300;
int front_black = 700;
int back_white = 500;
int back_black = 700;
float WLeft, WRight, WLeftR, WRightR, D, DL, DR, DX, DY, DT, linear_velocity, angular_velocity, X = 0.0, Y = 0.0, tim = 0.1;
float X_i = 0.0, Y_i = 0.0, theta_i = 0.0;
float r = 0.016, b = 0.095, C = 8100/*2300.0*/, theta = 0.0;
float ProLeft, DerLeft, EPLeft = 0.0, INTLeft, EILeft = 0.0;
float ProRight, DerRight, EPRight = 0.0, INTRight, EIRight = 0.0;
float GainLeft, GainRight;
unsigned long timeout_cmd_vel = 0;

//publisher         //topic name          //message
ros::Publisher line_front_pub("line_front", &Lin_front_msg);
ros::Publisher line_back_pub("line_back", &Lin_back_msg);
ros::Publisher front_distance_pub("front_distance", &front_distance_msg);
ros::Publisher right_distance_pub("right_distance", &right_distance_msg);
ros::Publisher left_distance_pub("left_distance", &left_distance_msg);
ros::Publisher pose_pub("pose", &pose_msg);
// SUBSCRIBER

/******************************************************/
// Defining the wheel velociy
float V = 0;
float W=0;
long t = millis();
/******************************************************/
// PID tuning parameters
float KP = 70;
float KI = 170;
float KD = 0.8;
/******************************************************/


/*  PERCEPTION      */
int sensL(int pin)
{
  int total = 0;
  for (int i = 0; i < 10; i++) {
    int sensor_value = analogRead(pin);
    int distance_cm = pow(3027.4 / sensor_value, 1.2134);
    total = distance_cm + total;
  }
  return total / 1000;
}
bool sensDF(int pin, int white, int black)
{
  int sensor_value = analogRead(pin);
  if (sensor_value >= black)
  {
    return false;
  }
  if (sensor_value <= white)
  {
    return true;
  }
}
long dDL = 0, dDLa = 0;
long dDR = 0, dDRa = 0;
void encoder()
{
  dDL = encL.read();
  dDR = encR.read();
  NL = dDL - dDLa;
  NR = dDR - dDRa;
}

/* KINEMATICS       */


void poseUpdate()
{
  DL = (2.0 * pi * r * (float)NL) / C;
  DR = (2.0 * pi * r * (float)NR) / C;
  D = (DL + DR) / 2.0;
  DT = (DR - DL) / b;
  DX = D * cos(DT);
  DY = D * sin(DT);
  X = X + DX;
  Y = Y + DY;
  theta = theta + DT;
  linear_velocity = ((PI * r) * (float)(NL + NR)) / ( C * tim);
  angular_velocity =  ( ( PI * r) * (float)(NL - NR)) / ( b * C * tim);
  WLeftR = (2.0 * linear_velocity - b * angular_velocity) / (2.0 * r);
  WRightR = (2.0 * linear_velocity + b * angular_velocity) / (2.0 * r);

  //Publishes pose
  pose_msg.x = X;
  pose_msg.y = Y;
  pose_msg.theta = theta;
  pose_pub.publish(&pose_msg);
}
/* CONTROLLER */
void pid_controller()
{
  ProLeft = WLeft - WLeftR;
  DerLeft = ProLeft  - EPLeft;
  INTLeft = INTLeft  + ProLeft;
  GainLeft = KP * ProLeft + KI * INTLeft * tim + (KD * (DerLeft / tim));
  EPLeft  = ProLeft;
  /*if (V == 0.0 && W == 0.0)
  {
    ProLeft = 0;
    DerLeft = 0;
    INTLeft = 0;
  }*/
  ProRight = WRight - WRightR;
  DerRight = ProRight  - EPRight;
  INTRight = EIRight + ProRight;
  GainRight = KP * ProRight + KI * INTRight * tim + (KD * (DerRight / tim));
  EPRight = ProRight;
  EIRight  = INTRight;
  /*if (V == 0.0 && W == 0.0)
  {
    ProRight = 0;
    DerRight = 0;
    INTRight = 0;
  }*/
}
void leftWheel()
{
  if (fabs(GainLeft) > 255)
    GainLeft = (fabs(GainLeft) / GainLeft) * 255;

  if (GainLeft > 0)
  {
    analogWrite(PWM1, (int)( GainLeft));
    digitalWrite(DirLeft1, HIGH);
    digitalWrite(DirLeft2, LOW);
  }
  else
  {
    analogWrite(PWM1, (int)(fabs(GainLeft)));
    digitalWrite(DirLeft2, HIGH);
    digitalWrite(DirLeft1, LOW);
  }
}
void rightWheel()
{
  if (fabs(GainRight) > 255)
    GainRight = (fabs(GainRight) / GainRight) * 255;

  if (GainRight > 0)
  {
    analogWrite(PWM2, (int)(GainRight)); // gain of PID
    digitalWrite(DirRight1, HIGH);
    digitalWrite(DirRight2, LOW);
  }
  else
  {
    analogWrite(PWM2, (int)(fabs( GainRight)));
    digitalWrite(DirRight2, HIGH);
    digitalWrite(DirRight1, LOW);
  }
}
void cmd_vel2wheel_CB(const geometry_msgs::Twist& cmd_vel_msg)
{
  nh.loginfo("inside call back");
  //Subscribes cmd_vel
  V = cmd_vel_msg.linear.x;
  W = cmd_vel_msg.angular.z;
  if(V == 0.0 && W == 0.0) {
    ProLeft = 0.0;
    DerLeft = 0.0;
    INTLeft = 0.0;
    ProRight = 0.0;
    DerRight = 0.0;
    INTRight = 0.0;
    GainLeft = 0.0;
    GainRight = 0.0;
  }
  WLeft = (2.0 * V - b * W) / (2.0 * r);
  WRight = (2.0 * V + b * W) / (2.0 * r);

  timeout_cmd_vel = millis();
}
void initial_pose_callback(const geometry_msgs::Pose2D& initial_pose_msg)
{
  X_i = initial_pose_msg.x;
  Y_i = initial_pose_msg.y;
  theta_i = initial_pose_msg.theta;
}
// DoubT//
void rgb_callback(const std_msgs::UInt8MultiArray& rgb_leds_msg)
{
  pixels.setPixelColor(0, pixels.Color(rgb_leds_msg.data[0], rgb_leds_msg.data[1], rgb_leds_msg.data[2]));
  pixels.show();
  pixels.setPixelColor(1, pixels.Color(rgb_leds_msg.data[3], rgb_leds_msg.data[4], rgb_leds_msg.data[5]));
  pixels.show();
 
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", cmd_vel2wheel_CB);
ros::Subscriber<geometry_msgs::Pose2D> initial_pose_sub("/initial_pose", initial_pose_callback);
ros::Subscriber<std_msgs::UInt8MultiArray> rgb_leds_sub("/rgb_leds", rgb_callback);

void setup()
{

  nh.initNode();

  nh.advertise(pose_pub);
  nh.advertise(line_front_pub);
  nh.advertise(line_back_pub);
  nh.advertise(front_distance_pub);
  nh.advertise(right_distance_pub);
  nh.advertise(left_distance_pub);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(initial_pose_sub);
  nh.subscribe(rgb_leds_sub);
  
  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  pinMode(DirLeft1, OUTPUT);
  pinMode(DirLeft2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DirRight1, OUTPUT);
  pinMode(DirRight2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  pixels.setPixelColor(1, pixels.Color(255, 0, 0));
  pixels.show();
  rgb_leds_msg.data_length = 6;
  
}

/*   MAIN    */
void loop()
{


  if ( (millis() - t) > 100)
  {
    t = millis();

    left_distance_msg.data = sensL(A2);
    left_distance_pub.publish(&left_distance_msg);

    right_distance_msg.data = sensL(A4);
    right_distance_pub.publish(&right_distance_msg);

    front_distance_msg.data = sensL(A3);
    front_distance_pub.publish(&front_distance_msg);

    Lin_front_msg.data = sensDF(A5, front_white, front_black);
    line_front_pub.publish(&Lin_front_msg);

    Lin_back_msg.data = sensDF(A6, back_white, back_black);
    line_back_pub.publish(&Lin_back_msg);

    encoder();
    //cmd_vel2wheel();
    poseUpdate();
    pid_controller();
    leftWheel();
    rightWheel();
    dDLa = dDL;
    dDRa = dDR;

  }

  nh.spinOnce();
}


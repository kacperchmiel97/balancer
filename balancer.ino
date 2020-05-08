#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16MultiArray.h>
#include <balancer/MotorControl.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DynamixelMotor.h>

void setMotorSpeed(const std_msgs::Int16MultiArray &msg);

/* WiFi connection stuff */
char ssid[30]= "balancer";
IPAddress my_address= IPAddress(192, 168, 1, 125);
uint16_t ros_port= 11411;
IPAddress ros_address= IPAddress(192, 168, 1, 127);

/* ROS stuff */
ros::NodeHandle nh;

std_msgs::String str_msg;
sensor_msgs::Imu imu_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher imu_publisher("imu/raw", &imu_msg);
ros::Subscriber<std_msgs::Int16MultiArray> motor_subscriber("motorSpeed", &setMotorSpeed);


/* MPU stuff */
Adafruit_MPU6050 mpu;

/* Dynamixel stuff */
#define MOTOR_TX_PIN D6
#define MOTOR_RX_PIN D7
#define MOTOR_DIR_PIN D5
#define MOTOR_BAUDRATE 115200
#define MOTOR_L_ID 2
#define MOTOR_R_ID 1

SoftwareDynamixelInterface interface(MOTOR_RX_PIN, MOTOR_TX_PIN, MOTOR_DIR_PIN);
DynamixelMotor motorL(interface, MOTOR_L_ID);
DynamixelMotor motorR(interface, MOTOR_R_ID);

/* ------------------------------------------------------------------------- */
/* Initialization */

void initConnection(char *ssid, IPAddress my_addr){
  WiFi.mode(WIFI_AP);
    
  Serial.println(); 
  Serial.print("Setting up access point ");
  Serial.println(ssid);
  
  WiFi.softAPConfig(my_addr, IPAddress(192,168,1,100), IPAddress(255,255,255,0));
    
  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(ssid) ? "Ready" : "Failed!");

  Serial.println("IP address: ");
  Serial.println(my_addr);
}

void initRos(void){
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(imu_publisher);
  nh.subscribe(motor_subscriber);
}

void initImu(void){
  /* Initialize IMU */
  Serial.println("Initialize MPU6050");
  while(!mpu.begin())
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
}

void initMotors(unsigned long baudrate){
  interface.reverseDirection(true);
  interface.begin(baudrate);
  
  delay(100);
  
  motorL.init();
  motorL.enableTorque();
  motorL.wheelMode();

  motorL.init();
  motorL.enableTorque();
  motorL.wheelMode();
}

/* ------------------------------------------------------------------------- */
/* Utility functions */

void connectToRos(IPAddress address, uint16_t port){
  nh.getHardware()->setConnection(address, port);
}

void setMotorSpeed(const std_msgs::Int16MultiArray &msg){
  motorL.speed(msg.data[0]);
  motorR.speed(msg.data[1]);
}

void sendImuData(void){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  imu_msg.linear_acceleration.x= a.acceleration.x;
  imu_msg.linear_acceleration.y= a.acceleration.y;
  imu_msg.linear_acceleration.z= a.acceleration.z;
  imu_msg.angular_velocity.x= g.gyro.x;
  imu_msg.angular_velocity.y= g.gyro.y;
  imu_msg.angular_velocity.z= g.gyro.z;
  imu_msg.header.stamp= nh.now();

  imu_publisher.publish(&imu_msg);
}

/* ------------------------------------------------------------------------- */
/* Main program */

void setup() {
  Serial.begin(115200);

  initConnection(ssid, my_address);
  initRos();
  connectToRos(ros_address, ros_port);
  initImu();
  initMotors(MOTOR_BAUDRATE);
  

  Serial.println();
  Serial.println("Initialization complete!");
}

void loop() {
  
  if(nh.connected()){
    sendImuData();
  }
  nh.spinOnce();
}

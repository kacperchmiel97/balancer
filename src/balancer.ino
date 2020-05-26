#include <ESP8266WiFi.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <balancer/MotorControl.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DynamixelMotor.h>

//Wykomentowanie tej definicji spowoduje podłączenie się do istniejącej sieci
#define WIFI_CLIENTMODE

/* Parametry i zmienne */

double radToDeg;
float speed_setpoint, dir_bias;
float pitch, pitch_gyro;
float kp_tilt, kd_tilt;
float kp_speed, kd_speed, ki_speed;

void set_dir_bias(const std_msgs::Float32 &msg){
  dir_bias= msg.data;
}

void set_speed_setpoint(const std_msgs::Float32 &msg){
  speed_setpoint= msg.data;
}

void set_kp_tilt(const std_msgs::Float32 &msg){
  kp_tilt= msg.data;
}

void set_kd_tilt(const std_msgs::Float32 &msg){
  kd_tilt= msg.data;
}

void set_kp_speed(const std_msgs::Float32 &msg){
  kp_speed= msg.data;
}

void set_kd_speed(const std_msgs::Float32 &msg){
  kd_speed= msg.data;
}

void set_ki_speed(const std_msgs::Float32 &msg){
  ki_speed= msg.data;
}

/* Rzeczy związane z WiFi i połączeniem */
char ssid[30]= "balancer";
IPAddress my_address= IPAddress(192, 168, 1, 125);
uint16_t ros_port= 11411;
//IPAddress ros_address= IPAddress(192, 168, 1, 127);
IPAddress ros_address= IPAddress(192, 168, 1, 196);


/* Zmienne ROS */
ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
std_msgs::Float32 pid_speed_out, pid_tilt_out, angle;

ros::Subscriber<std_msgs::Float32> speed_setpoint_subscriber("speed_setpoint", &set_speed_setpoint);
ros::Subscriber<std_msgs::Float32> dir_bias_subscriber("direction", &set_dir_bias);

ros::Subscriber<std_msgs::Float32> kp_tilt_subscriber("pid/tilt/p", &set_kp_tilt);
ros::Subscriber<std_msgs::Float32> kd_tilt_subscriber("pid/tilt/d", &set_kd_tilt);

ros::Subscriber<std_msgs::Float32> kp_speed_subscriber("pid/speed/p", &set_kp_speed);
ros::Subscriber<std_msgs::Float32> kd_speed_subscriber("pid/speed/d", &set_kd_speed);
ros::Subscriber<std_msgs::Float32> ki_speed_subscriber("pid/speed/i", &set_ki_speed);

ros::Publisher speed_out_publisher("pid/speed/output", &pid_speed_out);
ros::Publisher tilt_out_publisher("pid/tilt/output", &pid_tilt_out);
ros::Publisher angle_publisher("angle", &angle);
ros::Publisher imu_publisher("imu/raw", &imu_msg);

/* Zmienne i definicje do IMU */
Adafruit_MPU6050 mpu;

/* Dynamixel stuff */
#define MOTOR_TX_PIN D6
#define MOTOR_RX_PIN D7
#define MOTOR_DIR_PIN D5
#define MOTOR_BAUDRATE 115200
#define MOTOR_L_ID 2
#define MOTOR_R_ID 1


/* Zmienne i definicje do sterowania napędem */
SoftwareDynamixelInterface interface(MOTOR_RX_PIN, MOTOR_TX_PIN, MOTOR_DIR_PIN);
DynamixelMotor motorL(interface, MOTOR_L_ID);
DynamixelMotor motorR(interface, MOTOR_R_ID);

/* ------------------------------------------------------------------------- */
/* Inicjalizacja */

#ifndef WIFI_CLIENTMODE
//Ustawiamy własny Acces Point
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
#else
//Podłączamy się do istniejącego Acces Pointa
void initConnection(void){
  Serial.println("Initializing Wifi connection...");
  
  WiFi.mode(WIFI_STA);
    WiFi.begin("ssid", "passwd");

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
     Serial.print(".");
   }
  
    Serial.println("");
   Serial.println("WiFi connected");
   Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}
#endif

void initRos(void){
  nh.initNode();
  
  nh.advertise(imu_publisher);
  nh.advertise(tilt_out_publisher);
  nh.advertise(speed_out_publisher);
  nh.advertise(angle_publisher);
  
  nh.subscribe(kp_tilt_subscriber);
  nh.subscribe(kd_tilt_subscriber);
  
  nh.subscribe(kp_speed_subscriber);
  nh.subscribe(kd_speed_subscriber);
  nh.subscribe(ki_speed_subscriber);

  nh.subscribe(speed_setpoint_subscriber);
  nh.subscribe(dir_bias_subscriber);
}

void initImu(void){
    Serial.println("Initializing MPU6050...");
  while(!mpu.begin())
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  Serial.println("Initialized MPU6050.");
}

void initMotors(unsigned long baudrate){
  Serial.println("Initializing motors...");
  
  interface.reverseDirection(true);
  interface.begin(baudrate);
  
  delay(100);
  
  motorL.init();
  motorL.enableTorque();
  motorL.wheelMode();

  motorL.init();
  motorL.enableTorque();
  motorL.wheelMode();
  
  Serial.println("Initialized motors.");
}

/* ------------------------------------------------------------------------- */
/* Funkcje pomocnicze */

void connectToRos(IPAddress address, uint16_t port){
  nh.getHardware()->setConnection(address, port);
}

void sendImuData(sensors_event_t a, sensors_event_t g){
  imu_msg.linear_acceleration.x= a.acceleration.x;
  imu_msg.linear_acceleration.y= a.acceleration.y;
  imu_msg.linear_acceleration.z= a.acceleration.z;
  imu_msg.angular_velocity.x= g.gyro.x;
  imu_msg.angular_velocity.y= g.gyro.y;
  imu_msg.angular_velocity.z= g.gyro.z;
  imu_msg.header.stamp= nh.now();

  imu_publisher.publish(&imu_msg);
}

#define alpha 0.1
float calculatePitch(){
  static float pitch= 0;
  float aX, aY, aZ, gY;
  float y;
  static unsigned long last_time= 0;
  unsigned long now;
  unsigned int dt;

  now= millis();
  dt= now - last_time;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sendImuData(a, g);

  aX= a.acceleration.x;
  aY= a.acceleration.y;
  aZ= a.acceleration.z;
  gY= g.gyro.y;
  pitch_gyro= gY;

  y= atan2((-aX) , sqrt(aY * aY + aZ * aZ));

  pitch= (1-alpha)*(pitch + gY * (float)dt/1000) + alpha*y;
  
  last_time= now;

  return pitch * radToDeg;
}

float pidTilt(float setpoint, float p){
  float error, response;
  float pval, dval;
  static unsigned long last_time= 0;
  unsigned long now;
  unsigned int dt;

  now= millis();
  dt= now - last_time;

  error= setpoint - p;

  pval= kp_tilt * error;
  dval= -kd_tilt * pitch_gyro * dt;

  last_time= now;

  response= pval + dval;
  if(response > 30)
    response= 30;
  if(response < -30)
    response= -30;

  return response;
}

#define i_limit 10
float pidSpeed(float setpoint, float s){
  float error, response;
  float pval, dval;
  static float ival= 0;
  static float last_error;
  static unsigned long last_time= 0;
  unsigned long now;
  unsigned int dt;

  now= millis();
  dt= now - last_time;

  error= setpoint - s;

  pval= kp_speed * error;
  dval= -kd_speed * (last_error - error) * dt;

  if(error > 0 && ival < i_limit){
    ival+= ki_speed * error * dt/100;
  }
  else if(error < 0 && ival > -i_limit){
    ival+= ki_speed * error * dt/100;
  }
  
  last_time= now;
  last_error= error;

  response= pval + dval + ival;
  if(response > 25)
    response= 25;
  if(response < -25)
    response= -25;

  return response;
}

void defaultParameters(void){ 
    kp_tilt= 2;
    kd_tilt= 2.5;

    kp_speed= 1.5;
    kd_speed= -0.5;
    ki_speed= 0.04;
}

/* ------------------------------------------------------------------------- */
/* Main program */

void setup() {
  Serial.begin(115200);
  Serial.println();

  #ifndef WIFI_CLIENTMODE
  initConnection(ssid, my_address);
  #else
  initConnection();
  #endif
  initRos();
  connectToRos(ros_address, ros_port);
  initImu();
  initMotors(MOTOR_BAUDRATE);

  radToDeg= 57296 / 1000;

  /*
  Serial.println("Connecting to ROS master...");
  while(!nh.connected())
    nh.spinOnce();
  Serial.println("Connected.");
  */

  defaultParameters();
  speed_setpoint= 0;
  dir_bias= 0;

  Serial.println();
  Serial.println("Initialization complete!");
}

void loop() {
  float resp, p, p_set;
  static float motorSpeed= 0;
  static float averageSpeed= 0;

  //Obliczanie, o jaki kąt się pochylić w celu osiągnięcia zadanej prędkości
  p= calculatePitch();
  p_set= pidSpeed(speed_setpoint, averageSpeed);

  //Obliczanie, jaką prędkość zadać silnikom w celu osiągnięcia takiego kąta
  motorSpeed= pidTilt(-p_set, p);

  //Dla sprzężenia zwrotnego prędkości robimy średnią ruchomą w celu wygładzenia
  averageSpeed-= averageSpeed / 20;
  averageSpeed+= motorSpeed / 20;

  //Wysyłanie danych do komputera monitorującego
  angle.data= p;
  pid_tilt_out.data= motorSpeed * 33;
  pid_speed_out.data= p_set;
  speed_out_publisher.publish(&pid_speed_out);
  tilt_out_publisher.publish(&pid_tilt_out);
  angle_publisher.publish(&angle);

  //Zadawanie prędkości dla silników (ma zakres -1023 do 1023, a nasz sygnał -30 do 30,
  //więc skalujemy przez +-33; dodajemy też prędkości wynikające z komendy skrętu
  motorL.speed(33 * motorSpeed + dir_bias);
  motorR.speed(-33 * motorSpeed + dir_bias);
  
  delay(1);
  if(nh.connected())
      nh.spinOnce();
}

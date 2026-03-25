// ROS2 SLAM Rover Node - RPLidar C1 + STS3215 Motor Control
#ifndef ESP32
#error This example runs on ESP32
#endif

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <LDS_RPLIDAR_C1.h>
#include <WiFi.h>
#include <SCServo.h>

#define SLAMTEC_RPLIDAR_C1

#define MOTOR_DEBUG false
#define TIMING_DEBUG false

// --- WiFi Configuration ---
char ssid[] = "ankush-chanda";
char password[] = "esp32robot";
char agent_ip[] = "10.42.0.1";
uint32_t agent_port = 8888;

// --- Pin Configuration ---
const uint8_t LIDAR_RX_PIN = 16;
const uint8_t LIDAR_TX_PIN = 17;  
HardwareSerial LidarSerial(2);
LDS_RPLIDAR_C1 lidar;

const uint8_t SERVO_RX_PIN = 18;
const uint8_t SERVO_TX_PIN = 19;
SMS_STS sts;

// --- ODOMETRY VARIABLES ---
#define LEFT_MOTOR_ID 1
#define RIGHT_MOTOR_ID 2
const float TRACK_WIDTH = 0.18;         
const float WHEEL_RADIUS = 0.033;       
const float STEPS_PER_REV = 4096.0;    

const float METERS_PER_TICK = (2.0 * PI * WHEEL_RADIUS) / 4096.0;

int16_t last_left_ticks = 0;
int16_t last_right_ticks = 0;
bool first_odom_reading = true;
unsigned long last_odom_time = 0;

float robot_x = 0.0;
float robot_y = 0.0;
float robot_theta = 0.0;
 
// --- micro-ROS entities ---
rcl_publisher_t scan_pub;
rcl_publisher_t odom_publisher;
rcl_subscription_t twist_sub;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__LaserScan laser_scan_msg;
nav_msgs__msg__Odometry odom_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

#define SCAN_SIZE 360
float scan_ranges[SCAN_SIZE];
volatile bool scan_data_ready = false;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.print("micro-ROS Failed status on line "); Serial.println(__LINE__);}}

int16_t calculate_tick_diff(int16_t current, int16_t previous) {
  int16_t diff = current - previous;
  if (diff > 2048) {
    diff -= 4096; 
  } else if (diff < -2048) {
    diff += 4096; 
  }
  return diff;
}

void calculate_odometry() {
  unsigned long current_time = millis();

  int16_t current_left_ticks = -sts.ReadPos(LEFT_MOTOR_ID); 
  int16_t current_right_ticks = sts.ReadPos(RIGHT_MOTOR_ID);

  if (first_odom_reading) {
    last_left_ticks = current_left_ticks;
    last_right_ticks = current_right_ticks;
    last_odom_time = current_time;
    first_odom_reading = false;
  return;
  }

  double dt = (current_time - last_odom_time) / 1000.0;

  int16_t delta_left_ticks = calculate_tick_diff(current_left_ticks, last_left_ticks);
  int16_t delta_right_ticks = calculate_tick_diff(current_right_ticks, last_right_ticks);

  last_left_ticks = current_left_ticks;
  last_right_ticks = current_right_ticks;
  last_odom_time = current_time;

  float d_left = delta_left_ticks * METERS_PER_TICK;
  float d_right = delta_right_ticks * METERS_PER_TICK;

  float d_center = (d_left + d_right) / 2.0;
  float delta_theta = (d_right - d_left) / TRACK_WIDTH;

  robot_x += d_center * cos(robot_theta);
  robot_y += d_center * sin(robot_theta);
  robot_theta += delta_theta;

  if (robot_theta > PI) robot_theta -= 2.0 * PI;
  if (robot_theta < -PI) robot_theta += 2.0 * PI;

  float linear_velocity = d_center / dt;
  float angular_velocity = delta_theta / dt;

  int64_t time_ns = rmw_uros_epoch_nanos();
  odom_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000);
  odom_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);

  odom_msg.pose.pose.position.x = robot_x;
  odom_msg.pose.pose.position.y = robot_y;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(robot_theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(robot_theta / 2.0);

  odom_msg.pose.covariance[0]  = 0.001;  // x
  odom_msg.pose.covariance[7]  = 0.001;  // y
  odom_msg.pose.covariance[35] = 0.001;  // yaw

  odom_msg.twist.twist.linear.x = linear_velocity;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = angular_velocity;

  odom_msg.twist.covariance[0]  = 0.001;  // vx
  odom_msg.twist.covariance[35] = 0.001;  // omega_z

  rcl_publish(&odom_publisher, &odom_msg, NULL);
}

void twist_callback(const void * msgin) {

  #if DEBUG_MOTORS
  Serial.println("Received /cmd_vel");
  #endif

  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float v = msg->linear.x;
  float omega = msg->angular.z;
  
  float v_left = v - (omega * TRACK_WIDTH / 2.0);
  float v_right = v + (omega * TRACK_WIDTH / 2.0);
  
  float circumference = 2.0 * PI * WHEEL_RADIUS;
  float steps_per_meter = STEPS_PER_REV / circumference;
  
  int left_speed_steps = (int)(v_left * steps_per_meter);
  int right_speed_steps = (int)(v_right * steps_per_meter);
  
  left_speed_steps = -left_speed_steps;
  left_speed_steps = constrain(left_speed_steps, -3400, 3400);
  right_speed_steps = constrain(right_speed_steps, -3400, 3400);

  int left_final = abs(left_speed_steps);
  if (left_speed_steps < 0) left_final |= (1 << 15);

  int right_final = abs(right_speed_steps);
  if (right_speed_steps < 0) right_final |= (1 << 15);

  sts.WriteSpe(LEFT_MOTOR_ID, left_final, 50);
  sts.WriteSpe(RIGHT_MOTOR_ID, right_final, 50); 
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
  if (distance_mm > 0 && distance_mm < 12000) {
    float dist_m = distance_mm / 1000.0;
      angle_deg = fmod(angle_deg, 360.0f);
      if (angle_deg < 0.0f) angle_deg += 360.0f;
      int index = (int)angle_deg;
    if(index >= 0 && index < SCAN_SIZE) {
      scan_ranges[index] = dist_m;
    }
  }
  if (scan_completed) {
    scan_data_ready = true;
  }
}

void publishLaserScan() {
  scan_data_ready = false;
  int64_t time_ns = rmw_uros_epoch_nanos();
  laser_scan_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000);
  laser_scan_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);
  
  for(int i = 0; i < SCAN_SIZE; i++) {
    laser_scan_msg.ranges.data[i] = scan_ranges[i];
  }
  
  rcl_publish(&scan_pub, &laser_scan_msg, NULL);

  for(int i = 0; i < SCAN_SIZE; i++) {
    scan_ranges[i] = 12.0;
  }
}

int lidar_serial_read_callback() { return LidarSerial.read(); }
size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) { return LidarSerial.write(buffer, length); }
void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {}

void setup() {
  Serial.begin(115200);
  delay(2000); 

  // --- 1. Init Servos ---
  Serial1.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  sts.pSerial = &Serial1;
  
  sts.unLockEprom(LEFT_MOTOR_ID);
  sts.WheelMode(LEFT_MOTOR_ID);
  sts.LockEprom(LEFT_MOTOR_ID);
  
  sts.unLockEprom(RIGHT_MOTOR_ID);
  sts.WheelMode(RIGHT_MOTOR_ID);
  sts.LockEprom(RIGHT_MOTOR_ID);

  // --- 2. Init WiFi ---
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  while(WiFi.status() != WL_CONNECTED) { delay(500); }

  // --- 3. Init micro-ROS ---
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "rover_esp32_node", "", &support));

  bool synced = false;
  for (int i = 0; i < 10 && !synced; i++) {
    if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
      synced = true;
    }
    delay(500);
  }
  
  // NOTE: Original Default Reliable QoS initialization
  RCCHECK(rclc_publisher_init_default(&scan_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan"));
  RCCHECK(rclc_subscription_init_default(&twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.header.frame_id.size = strlen("odom");
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;

  odom_msg.child_frame_id.data = (char*)"base_link";
  odom_msg.child_frame_id.size = strlen("base_link");
  odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;
  
  // NOTE: Original Executor Handle Count
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA));

  laser_scan_msg.header.frame_id.data = (char*)"laser_frame";
  laser_scan_msg.header.frame_id.size = strlen("laser_frame");
  laser_scan_msg.header.frame_id.capacity = strlen("laser_frame") + 1;
  laser_scan_msg.angle_min = 0.0;
  laser_scan_msg.angle_max = 2 * PI;
  laser_scan_msg.angle_increment = (2 * PI) / SCAN_SIZE;
  laser_scan_msg.range_min = 0.02;
  laser_scan_msg.range_max = 12.0;
  laser_scan_msg.ranges.data = scan_ranges;
  laser_scan_msg.ranges.capacity = SCAN_SIZE;
  laser_scan_msg.ranges.size = SCAN_SIZE;

  // --- 4. Init LiDAR ---
  LidarSerial.setRxBufferSize(1024);
  LidarSerial.begin(460800, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setPacketCallback(lidar_packet_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  
  lidar.init();
  lidar.start(); 
}

void loop() {
  unsigned long t1 = micros();
  lidar.loop();
  unsigned long t2 = micros();

  unsigned long now = millis();
  static unsigned long last_time = 0;

  if (now - last_time >= 100) {
    calculate_odometry();
    publishLaserScan();
    last_time = now;
  }

  unsigned long t3 = micros();
  rclc_executor_spin_some(&executor, 0);
  unsigned long t4 = micros();

  static unsigned long last_report = 0;
  #if TIMING_DEBUG
  if (now - last_report >= 1000) {
    Serial.print("lidar.loop()="); Serial.print(t2-t1);
    Serial.print("us  publish="); Serial.print(t3-t2);
    Serial.print("us  executor="); Serial.println(t4-t3);
    last_report = now;
  }
  #endif
}
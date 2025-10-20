/**
 * @file move_hit_node_NO_leftlimit.ino
 * @brief 敲擊機構控制程式（常開限位版本，左側限位）
 * @details
 * - X 軸左端為限位（常開型 NO）
 * - 壓下 HIGH → 表示到限位
 * - Homing 時往左跑直到 HIGH
 * - 壓下限位後設定 current_x = 0.0
 * - a → 向左 (X−)
 * - d → 向右 (X+)
 */

 #include <Arduino.h>
 #include <micro_ros_arduino.h>
 #include <stdio.h>
 #include <rcl/rcl.h>
 #include <rcl/error_handling.h>
 #include <rclc/rclc.h>
 #include <rclc/executor.h>
 #include <std_msgs/msg/string.h>
 #include <std_msgs/msg/bool.h>
 
 // ===================== ROS 2 定義 =====================
 rcl_subscription_t subscriber_led;
 rcl_subscription_t subscriber_coordinates;
 std_msgs__msg__Bool led_msg;
 std_msgs__msg__String coordinates_msg;
 std_msgs__msg__String top_base_locate_msg;
 rcl_subscription_t subscriber_top_base_locate;
 rcl_subscription_t subscriber_keyboard;
 std_msgs__msg__String keyboard_msg;
 
 rclc_executor_t executor;
 rclc_support_t support;
 rcl_allocator_t allocator;
 rcl_node_t node;
 
 rcl_publisher_t publisher_locate;
 std_msgs__msg__String locate_msg;
 rcl_publisher_t publisher_move_hit_initialized;
 std_msgs__msg__String move_hit_initialized_msg;
 
 // ===================== 硬體腳位設定 =====================
 #define LED_PIN 2
 bool base_locate_triggered = false;
 unsigned long base_locate_timestamp = 0;
 
 // 步進馬達腳位
 const int xenablePin = 18;
 const int yenablePin = 27;
 const int xstepPin   = 16;
 const int xdirPin    = 17;
 const int ystepPin   = 26;
 const int ydirPin    = 25;
 const int xleftPin   = 33;   // X 軸左端限位（常開，高觸發）
 const int ydownPin   = 32;   // Y 軸下端限位（常開，高觸發）
 
 // ===================== 步進參數 =====================
 const float steps_per_rotation = 200.0f * 16.0f;  // 1/16 微步
 const float hit_of_circle      = 14.0f;           // 絲桿或皮帶每圈距離(mm)
 const float max_x              = 350.0f;
 const float max_y              = 140.0f;
 
 // ===================== 位置 =====================
 float current_x = 0.0f;
 float current_y = 0.0f;
 
 // ===================== 敲擊機構 =====================
 const int motorPin1   = 21;
 const int motorPin2   = 22;
 const int limitSwitch = 13;  // 敲擊限位（常開，高觸發）
 bool motorRunning = false;
 
// ===================== 狀態與心跳 =====================
char current_status[16] = "initializing"; // 小寫：initializing/ready/moving/hitting/error
unsigned long last_heartbeat_ms = 0;
const unsigned long heartbeat_interval_ms = 1000; // 1s 心跳

 // ===================== 函數宣告 =====================
 void moveToTarget(float target_x, float target_y);
 void hitOnce();
 void initializeAxes();
void set_status(const char* s);
void publish_position_now();
 
 // ===================== 步進控制 =====================
 static inline void pulseStep(int stepPin) {
   digitalWrite(stepPin, HIGH);
   delayMicroseconds(500);
   digitalWrite(stepPin, LOW);
   delayMicroseconds(500);
 }
 
 void stepMotor(int stepPin, int dirPin, bool direction) {
   digitalWrite(dirPin, direction ? HIGH : LOW);
   pulseStep(stepPin);
 }
 
 // ===================== Homing 功能 =====================
 void homeAxis(int stepPin, int dirPin, int limitPin, bool directionPositive, float &axis_pos, float home_value) {
   // 若上電時已壓著，先退開
   if (digitalRead(limitPin) == HIGH) {
     Serial.println("限位一開始被壓著，先退開...");
     digitalWrite(dirPin, directionPositive ? LOW : HIGH);
     for (int i = 0; i < 500; i++) {
       pulseStep(stepPin);
       delayMicroseconds(800);
     }
   }
 
   // 開始 homing（常開：HIGH = 壓下）
   digitalWrite(dirPin, directionPositive ? HIGH : LOW);
   while (digitalRead(limitPin) == LOW) {  // 未壓下時持續移動
     pulseStep(stepPin);
     delayMicroseconds(800);
   }
 
   // 退回少許避免壓著
   digitalWrite(dirPin, directionPositive ? LOW : HIGH);
   for (int i = 0; i < 200; i++) {
     pulseStep(stepPin);
     delayMicroseconds(800);
   }
 
   // 設定座標
   axis_pos = home_value;
   Serial.print("Homing 完成，軸座標 = ");
   Serial.println(axis_pos);
 }
 
 void initializeAxes() {
   // X 軸往左 homing，壓到左邊限位 = 0 mm
   homeAxis(xstepPin, xdirPin, xleftPin, false, current_x, 0.0f);
 
   // Y 軸往下 homing，壓到下方限位 = 0 mm
   homeAxis(ystepPin, ydirPin, ydownPin, false, current_y, 0.0f);
 
   Serial.println("Homing 完成！");
 }
 
 // ===================== 敲擊馬達初始化 =====================
void initializeHitMotor() {
    Serial.println("執行敲擊馬達初始化...");
  
    // 確保初始狀態為停止
    stopMotor();
  
    // 朝敲擊方向旋轉，直到碰到限位（HIGH）
    digitalWrite(motorPin1, HIGH);  // 敲擊方向
    digitalWrite(motorPin2, LOW);
  
    unsigned long startTime = millis();
  
    while (digitalRead(limitSwitch) == LOW) {  // 常開：LOW = 未壓
      delay(5);
      if (millis() - startTime > 4000) {
        Serial.println("敲擊馬達初始化超時（未壓到限位）");
        stopMotor();
        return;
      }
    }
  
    stopMotor();
    Serial.println("敲擊馬達回到原點（已壓到限位）");
  }
  
 // ===================== 移動控制 =====================
 void moveToTarget(float target_x, float target_y) {
   if (target_x < 0.0f || target_x > max_x || target_y < 0.0f || target_y > max_y) return;
 
  set_status("moving");

   float x_steps_f = (target_x - current_x) * (steps_per_rotation / hit_of_circle);
   float y_steps_f = (target_y - current_y) * (steps_per_rotation / hit_of_circle);
   int x_steps = (int)(x_steps_f >= 0 ? floorf(x_steps_f + 0.5f) : ceilf(x_steps_f - 0.5f));
   int y_steps = (int)(y_steps_f >= 0 ? floorf(y_steps_f + 0.5f) : ceilf(y_steps_f - 0.5f));
 
   for (int i = 0; i < abs(x_steps); i++) stepMotor(xstepPin, xdirPin, x_steps > 0);
   for (int i = 0; i < abs(y_steps); i++) stepMotor(ystepPin, ydirPin, y_steps > 0);
 
   current_x = target_x;
   current_y = target_y;
  publish_position_now();
  set_status("ready");
 }
 
 // ===================== 敲擊控制 =====================
 void startMotor() {
   digitalWrite(motorPin1, HIGH);
   digitalWrite(motorPin2, LOW);
 }
 void stopMotor() {
   digitalWrite(motorPin1, LOW);
   digitalWrite(motorPin2, LOW);
 }

// ===================== 狀態/位置回報 =====================
void set_status(const char* s) {
  // 限制長度，並保持小寫格式
  size_t n = strlen(s);
  if (n >= sizeof(current_status)) n = sizeof(current_status) - 1;
  memcpy(current_status, s, n);
  current_status[n] = '\0';
  // 立刻回報一次狀態
  char buf[48];
  int len = snprintf(buf, sizeof(buf), "status:%s", current_status);
  if (len < 0) return;
  publish_locate_text(buf);
}

void publish_position_now() {
  char buf[64];
  int len = snprintf(buf, sizeof(buf), "pos:%.2f,%.2f", current_x, current_y);
  if (len < 0) return;
  publish_locate_text(buf);
}
 
 // ===================== ROS 輔助函式 =====================
 void publish_locate_text(const char* txt) {
   size_t n = strlen(txt);
   if (n >= locate_msg.data.capacity) n = locate_msg.data.capacity - 1;
   memcpy(locate_msg.data.data, txt, n);
   locate_msg.data.data[n] = '\0';
   locate_msg.data.size = n;
   rcl_publish(&publisher_locate, &locate_msg, NULL);
 }
 void publish_initialized_ok() {
   const char* ok = "OK";
   memcpy(move_hit_initialized_msg.data.data, ok, 2);
   move_hit_initialized_msg.data.data[2] = '\0';
   move_hit_initialized_msg.data.size = 2;
   rcl_publish(&publisher_move_hit_initialized, &move_hit_initialized_msg, NULL);
 }
 
 // ===================== ROS2 回調 =====================
 void subscription_led_callback(const void *msgin) {
   const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
   digitalWrite(LED_PIN, msg->data);
 }
 void subscription_coordinates_callback(const void *msgin) {
   const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
   float x, y;
   if (sscanf(msg->data.data, "%f,%f", &x, &y) == 2) moveToTarget(x, y);
 }
 void subscription_top_base_locate_callback(const void *msgin) {
   const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
   if (strcmp(msg->data.data, "Start") == 0) {
     base_locate_triggered = true;
     base_locate_timestamp = millis();
   }
 }
 
 // 鍵盤控制：a 向左、d 向右
 void subscription_keyboard_callback(const void * msgin) {
   const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
   if (msg->data.size < 1) return;
   char key = msg->data.data[0];
 
   switch (key) {
     case 'w': moveToTarget(current_x, current_y + 10.0f); break; // Y+
     case 's': moveToTarget(current_x, current_y - 10.0f); break; // Y-
     case 'a': moveToTarget(current_x - 10.0f, current_y); break; // ← 向左 (X-)
     case 'd': moveToTarget(current_x + 10.0f, current_y); break; // → 向右 (X+)
     case 'h': hitOnce(); break;
     case 'r': moveToTarget(0.0f, 0.0f); break;
     case 'i': initializeAxes(); break;
     default: break;
   }
 }
 
 // ===================== 初始化 =====================
 void setup() {
   Serial.begin(115200);
   set_microros_transports();
 
   pinMode(LED_PIN, OUTPUT);
   pinMode(xstepPin, OUTPUT);
   pinMode(xdirPin, OUTPUT);
   pinMode(ystepPin, OUTPUT);
   pinMode(ydirPin, OUTPUT);
   pinMode(xenablePin, OUTPUT);
   pinMode(yenablePin, OUTPUT);
   digitalWrite(xenablePin, LOW);
   digitalWrite(yenablePin, LOW);
 
   // 常開型限位使用 PULLDOWN
   pinMode(xleftPin, INPUT_PULLDOWN);
   pinMode(ydownPin, INPUT_PULLDOWN);
   pinMode(limitSwitch, INPUT_PULLDOWN);
 
   pinMode(motorPin1, OUTPUT);
   pinMode(motorPin2, OUTPUT);
   digitalWrite(motorPin1, LOW);
   digitalWrite(motorPin2, LOW);
 
   // ROS 初始化
   allocator = rcl_get_default_allocator();
   rclc_support_init(&support, 0, NULL, &allocator);
   rclc_node_init_default(&node, "move_hit_node", "", &support);
   //訂閱話題
   rclc_subscription_init_default(&subscriber_led, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/ros2_led");
   rclc_subscription_init_default(&subscriber_top_base_locate, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/top_base_locate");
   rclc_subscription_init_default(&subscriber_coordinates, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/World_Coordinates");
   rclc_subscription_init_default(&subscriber_keyboard, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/keyboard_control");
   //發佈話題
   rclc_publisher_init_default(&publisher_locate, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/moveknock_locate_node");
   rclc_publisher_init_default(&publisher_move_hit_initialized, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/move_hit_initialized");
 
   //初始化話題
   coordinates_msg.data.data = (char *)malloc(100);
   coordinates_msg.data.size = 0;
   coordinates_msg.data.capacity = 100;
   move_hit_initialized_msg.data.data = (char *)malloc(50);
   move_hit_initialized_msg.data.size = 0;
   move_hit_initialized_msg.data.capacity = 50;
   top_base_locate_msg.data.data = (char *)malloc(100);
   top_base_locate_msg.data.size = 0;
   top_base_locate_msg.data.capacity = 100;
   locate_msg.data.data = (char *)malloc(100);
   locate_msg.data.size = 0;
   locate_msg.data.capacity = 100;
   keyboard_msg.data.data = (char *)malloc(10);
   keyboard_msg.data.size = 0;
   keyboard_msg.data.capacity = 10;
 
   rclc_executor_init(&executor, &support.context, 4, &allocator);
   rclc_executor_add_subscription(&executor, &subscriber_led, &led_msg, &subscription_led_callback, ON_NEW_DATA);
   rclc_executor_add_subscription(&executor, &subscriber_coordinates, &coordinates_msg, &subscription_coordinates_callback, ON_NEW_DATA);
   rclc_executor_add_subscription(&executor, &subscriber_top_base_locate, &top_base_locate_msg, &subscription_top_base_locate_callback, ON_NEW_DATA);
   rclc_executor_add_subscription(&executor, &subscriber_keyboard, &keyboard_msg, &subscription_keyboard_callback, ON_NEW_DATA);
 
     // 通電後自動初始化 XY 軸與敲擊機構
  initializeAxes();
  initializeHitMotor();

  // 發佈「上線完成」
  publish_initialized_ok();
  set_status("ready");
  publish_position_now();

 }
 
 // ===================== 主迴圈 =====================
 void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  unsigned long now_ms = millis();
  if (now_ms - last_heartbeat_ms >= heartbeat_interval_ms) {
    last_heartbeat_ms = now_ms;
    // 心跳：狀態 + 位置
    char sbuf[48];
    int slen = snprintf(sbuf, sizeof(sbuf), "status:%s", current_status);
    if (slen > 0) publish_locate_text(sbuf);
    publish_position_now();
  }
 }
 
 // ===================== 敲擊流程 =====================
// ===================== 敲擊一次（與初始化方向相同） =====================
void hitOnce() {
    if (motorRunning) return;
    motorRunning = true;
  
    Serial.println("敲擊動作開始");
    publish_locate_text("start_recording");
    set_status("hitting");
  
    // 朝敲擊方向旋轉（與初始化相同）
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  
    unsigned long startTime = millis();
  
    while (digitalRead(limitSwitch) == LOW) {  // 常開：LOW = 未壓
      delay(2);
      if (millis() - startTime > 4000) {
        Serial.println("敲擊動作超時（未碰到限位）");
        stopMotor();
        motorRunning = false;
        publish_locate_text("stop_recording");
        set_status("error");
        return;
      }
    }
  
    // 一旦壓到限位，馬達停止
    stopMotor();
    motorRunning = false;
    publish_locate_text("stop_recording");
    Serial.println("敲擊完成（已碰到限位停止）");
    set_status("ready");
  }
  
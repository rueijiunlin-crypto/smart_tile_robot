#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include "math.h"

// 訂閱者和消息變量
rcl_subscription_t subscriber_led;
rcl_subscription_t subscriber_move;
rcl_subscription_t subscriber_keyboard_input;  
rcl_publisher_t publisher_locate;  // Publisher for /top_base_locate
rcl_publisher_t publisher_coordinates;  // Publisher for /top_base_Coordinates
std_msgs__msg__String locate_msg;  
std_msgs__msg__String coordinates_msg; 
rcl_subscription_t subscriber_move_hit_initialized;
std_msgs__msg__String move_hit_initialized_msg;

std_msgs__msg__Bool led_msg;
std_msgs__msg__String move_msg;
std_msgs__msg__String keyboard_input_msg;  // 鍵盤輸入的消息變量

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// 定義針腳
const int xstepPin = 16;
const int xdirPin = 17;
const int ystepPin = 26;
const int ydirPin = 25;
#define LED_PIN 2

static unsigned long lastMoveTime = 0;
const unsigned long moveInterval = 100; // 設定移動間隔為 100 毫秒

// 步進電機參數
const float steps_per_rotation = 200 * 16; // 1/16 步進模式下的每轉步數
const float ropeLength_of_circle = 7.5;   // 每轉繩長 (cm)
const float x_correction_factor = 2; // 根据实验计算得到的修正x比例

// 機器人初始坐標
int current_x = 0;
int current_y = 48;
const int w = 79; // 卷軸之間的距離
float ystart_long=sqrt(pow(w,2)+pow(current_y,2));
float xstart_long=current_y;

// 用於比較的繩長變量
float old_L1 = 0.0;
float old_L2 = 0.0;


// 檢查坐標是否在範圍內
bool check_coords_matrix(int x, int y) {
  return (x >= 0 && x <= 79) && (y >= 0 && y <= 49); // 最大範圍 (50,82)
}

// 計算 L1 變化
bool first_time1 = true;
float calculate_L1(int x, int y) {
  float new_L1 = sqrt(pow(x, 2) + pow(y, 2)); // 計算新的 L1 長度
  float delta_L1 = 0.0;

  if (first_time1) {
    delta_L1 = new_L1 - xstart_long; // 從初始點的變化
    first_time1 = false;
  } else {
    delta_L1 = new_L1 - old_L1; // 計算 L1 變化
  }

  old_L1 = new_L1; // 更新舊的 L1 長度
  return delta_L1;
}

// 計算 L2 變化
bool first_time2 = true;
float calculate_L2(int x, int y) {
  float new_L2 = sqrt(pow(x, 2) + pow(y, 2) + pow(w, 2) - 2 * w * x); // 計算新的 L2 長度
  float delta_L2 = 0.0;

  if (first_time2) {
    delta_L2 = new_L2 - ystart_long; // 從初始點的變化
    first_time2 = false;
  } else {
    delta_L2 = new_L2 - old_L2; // 計算 L2 變化
  }

  old_L2 = new_L2; // 更新舊的 L2 長度
  return delta_L2;
}

// 送脈衝函數（發送單一脈衝）
void sendSinglePulse(int stepPin) {
  digitalWrite(stepPin, HIGH); // 設置步進腳為高電平
  delayMicroseconds(100);      // 脈衝寬度，可根據需要調整
  digitalWrite(stepPin, LOW);  // 設置步進腳為低電平
  delayMicroseconds(100);      // 延遲，控制脈衝頻率
}

// 設置目標位置
void setTargetPositions(float delta_L1, float delta_L2) {
  // 計算步數
  int steps_x = abs(delta_L1 *x_correction_factor* steps_per_rotation / ropeLength_of_circle)*10;
  int steps_y = abs(delta_L2 * steps_per_rotation / ropeLength_of_circle)*10;

  // 計算每步的延遲，以便在相同的時間內完成移動
  unsigned long pulseDelay_x = (steps_x > 0) ? (500000 / steps_x) : 0; // 以微秒計
  unsigned long pulseDelay_y = (steps_y > 0) ? (500000 / steps_y) : 0; // 以微秒計

  // 設置方向
  bool direction_x = delta_L1 >= 0;
  bool direction_y = delta_L2 <= 0;

  digitalWrite(xdirPin, direction_x); // 設置 X 軸方向
  digitalWrite(ydirPin, direction_y); // 設置 Y 軸方向

  // 送出脈衝給兩個電機
  int max_steps = max(steps_x, steps_y);

  for (int i = 0; i < max_steps; i++) {
    if (i < steps_x) {
      digitalWrite(xstepPin, HIGH);
      delayMicroseconds(pulseDelay_x);
      digitalWrite(xstepPin, LOW);
      delayMicroseconds(pulseDelay_x);
    }

    if (i < steps_y) {
      digitalWrite(ystepPin, HIGH);
      delayMicroseconds(pulseDelay_y);
      digitalWrite(ystepPin, LOW);
      delayMicroseconds(pulseDelay_y);
    }
  }
}

// 訂閱 /ros_led 的回調
void subscription_led_callback(const void *msg_in) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
  if (msg->data) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
  }
}

// 訂閱 /move2 的回調
void subscription_move_callback(const void *msg_in) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;

  // 添加時間間隔判斷
  if (millis() - lastMoveTime > moveInterval) {
    // 處理移動命令
    if (strcmp(msg->data.data, "w") == 0 || strcmp(msg->data.data, "t") == 0) {
      current_y -= 1;
    } else if (strcmp(msg->data.data, "s") == 0 || strcmp(msg->data.data, "g")==0) {
      current_y += 1;
    } else if (strcmp(msg->data.data, "a") == 0) {
      current_x -= 1;
    } else if (strcmp(msg->data.data, "d") == 0) {
      current_x += 1;
    }
    move_msg.data.size = 0;
    coordinates_msg.data.size = 0;
    locate_msg.data.size = 0;
    // 檢查坐標是否有效
    if (check_coords_matrix(current_x, current_y)) {
      float delta_L1 = calculate_L1(current_x, current_y);
      float delta_L2 = calculate_L2(current_x, current_y);
      setTargetPositions(delta_L1, delta_L2);
      lastMoveTime = millis(); // 更新上一次移動時間

      // Publish to /top_base_locate after moving
      snprintf(locate_msg.data.data, 100, "Located at (%d, %d)", current_x, current_y);
      locate_msg.data.size = strlen(locate_msg.data.data); // 更新消息大小
      rcl_publish(&publisher_locate, &locate_msg, NULL); // 發布消息
    } else {
      Serial.println("輸入的座標超出活動空間");
    }
  }
}

// 命令佇列相關變量
#define COMMAND_QUEUE_SIZE 100
char commandQueue[COMMAND_QUEUE_SIZE];
int commandQueueHead = 0;
int commandQueueTail = 0;

// 檢查佇列是否為空
bool isQueueEmpty() {
  return commandQueueHead == commandQueueTail;
}

// 檢查佇列是否已滿
bool isQueueFull() {
  return ((commandQueueTail + 1) % COMMAND_QUEUE_SIZE) == commandQueueHead;
}

// 將命令添加到佇列
bool enqueueCommand(char command) {
  if (!isQueueFull()) {
    commandQueue[commandQueueTail] = command;
    commandQueueTail = (commandQueueTail + 1) % COMMAND_QUEUE_SIZE;
    return true;
  }
  return false; // 佇列已滿
}

// 從佇列中取出一個命令
bool dequeueCommand(char *command) {
  if (!isQueueEmpty()) {
    *command = commandQueue[commandQueueHead];
    commandQueueHead = (commandQueueHead + 1) % COMMAND_QUEUE_SIZE;
    return true;
  }
  return false; // 佇列為空
}

// 訂閱 /keyboard_input 的回調
void subscription_keyboard_input_callback(const void *msg_in) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
  // 確認接收到的字串長度大於0，避免空字串
  if (msg->data.size > 0) {
    // 確保字串以 '\0' 結尾
    if (msg->data.size < 100) { // 防止溢出
      msg->data.data[msg->data.size] = '\0';
    }

    // 打印接收到的完整字串，作為調試
    Serial.print("Received command string: ");
    Serial.println(msg->data.data);

    // 遍歷字串，將每個字母添加到命令佇列
    for (size_t i = 0; i < msg->data.size; i++) {
      char command = msg->data.data[i];  // 取出當前的字母命令

      // 打印每個字母命令，調試用
      Serial.print("Enqueuing command: ");
      Serial.println(command);

      if (!enqueueCommand(command)) {
        Serial.println("Command queue is full. Command skipped.");
      }
    }
  } else {
    Serial.println("收到的命令字串為空");
  }
  keyboard_input_msg.data.size = 0;
}

void subscription_move_hit_initialized_callback(const void *msg_in) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;

  if (msg->data.size > 0) {
    msg->data.data[msg->data.size] = '\0'; // Ensure null-terminated string

    // Parse the command
    char command = msg->data.data[0]; // Expect 'a' or 'd'
    int steps = strlen(msg->data.data); // Number of repeated characters

    if (command == 'a' || command == 'd') {
      Serial.print("Command: ");
      Serial.print(command);
      Serial.print(", Steps: ");
      Serial.println(steps);

      // Set direction
      bool direction = (command == 'a') ? LOW : HIGH;
      digitalWrite(xdirPin, direction);

      // Move the motor
      for (int i = 0; i < steps; i++) {
        sendSinglePulse(xstepPin);
        delay(50); // Adjust delay for motor speed
      }

      Serial.println("Motor movement complete.");
    } else {
      Serial.println("Invalid command in /move_hit_initialized");
    }
  }
}

void setup() {
  // 初始化針腳
  pinMode(xstepPin, OUTPUT);
  pinMode(xdirPin, OUTPUT);
  pinMode(ystepPin, OUTPUT);
  pinMode(ydirPin, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  // 初始化串口
  Serial.begin(115200);
  //set_microros_transports(); 
  //set_microros_wifi_transports("NKUSTCED_2.4G", "nkustced22283", "192.168.50.249", 2024);
  set_microros_wifi_transports("手機網路ip", "密碼", "ip", 2024);


  // 初始化 micro-ROS 組件
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "move_node", "", &support);

  // 初始化訂閱者
  rclc_subscription_init_default(&subscriber_led, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/ros2_led");
  rclc_subscription_init_default(&subscriber_move, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/move2");
  rclc_subscription_init_default(&subscriber_keyboard_input, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/keyboard_input");
  rclc_publisher_init_default(&publisher_locate, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/top_base_locate");
  rclc_publisher_init_default(&publisher_coordinates, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/top_base_Coordinates");
  rclc_subscription_init_default(&subscriber_move_hit_initialized, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/move_hit_initialized");

  // 分配內存並初始化消息
  locate_msg.data.data = (char*)malloc(100 * sizeof(char)); // 分配 100 字符的內存
  locate_msg.data.size = 0;
  locate_msg.data.capacity = 100;
  
  coordinates_msg.data.data = (char *)malloc(100 * sizeof(char));
  coordinates_msg.data.size = 0;
  coordinates_msg.data.capacity = 100;

  move_msg.data.data = (char*)malloc(100 * sizeof(char)); // 分配 100 字符的內存
  move_msg.data.size = 0;
  move_msg.data.capacity = 100;

  keyboard_input_msg.data.data = (char*)malloc(1000 * sizeof(char)); // 分配 100 字符的內存
  keyboard_input_msg.data.size = 0;
  keyboard_input_msg.data.capacity = 1000;

  move_hit_initialized_msg.data.data = (char*)malloc(100 * sizeof(char)); // Allocate memory for 100 characters
  move_hit_initialized_msg.data.size = 0;
  move_hit_initialized_msg.data.capacity = 100;
  
  // 初始化 executor
  rclc_executor_init(&executor, &support.context, 4, &allocator);  // 訂閱者總數 4
  rclc_executor_add_subscription(&executor, &subscriber_led, &led_msg, &subscription_led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_move, &move_msg, &subscription_move_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_keyboard_input, &keyboard_input_msg, &subscription_keyboard_input_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_move_hit_initialized, &move_hit_initialized_msg, &subscription_move_hit_initialized_callback, ON_NEW_DATA);
}

void loop() {
  // 處理 ROS 訂閱者
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // 處理命令佇列
  static bool isProcessing = false; // 是否正在處理命令

  if (!isProcessing && !isQueueEmpty()) {
    char command;
    if (dequeueCommand(&command)) {
      isProcessing = true;

      Serial.print("Processing command: ");
      Serial.println(command);

      // 執行移動命令
      if (command == 'w' || command == 't') {
        current_y -= 1;  // 向上移動
      } else if (command == 's' || command == 'g') {
        current_y += 1;  // 向下移動
      } else if (command == 'a') {
        current_x -= 1;  // 向左移動
      } else if (command == 'd') {
        current_x += 1;  // 向右移動
      }

      // 檢查移動後的坐標是否有效
      if (check_coords_matrix(current_x, current_y)) {
        float delta_L1 = calculate_L1(current_x, current_y);
        float delta_L2 = calculate_L2(current_x, current_y);
        setTargetPositions(delta_L1, delta_L2);

        // 發佈當前位置到 /top_base_locate，這只觸發於 /move2 命令
        snprintf(locate_msg.data.data, 100, "Located at (%d, %d)", current_x, current_y);
        locate_msg.data.size = strlen(locate_msg.data.data);
        rcl_publish(&publisher_locate, &locate_msg, NULL);

        Serial.print("Moved to: ");
        Serial.print(current_x);
        Serial.print(", ");
        Serial.println(current_y);
      } else {
        Serial.println("輸入的座標超出活動空間");
      }

      // 更新移動時間
      lastMoveTime = millis();

      // 非阻塞延遲以處理其他訂閱者
      unsigned long processStartTime = millis();
      while (millis() - processStartTime < moveInterval) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }

      isProcessing = false;
    }
  }

  // 獨立發布當前坐標到 /top_base_Coordinates
  static unsigned long lastCoordinatePublishTime = 0;
  if (millis() - lastCoordinatePublishTime >= 1000) { // 每秒發布一次
    snprintf(coordinates_msg.data.data, 100, "Current coordinates: (%d, %d)", current_x, current_y);
    coordinates_msg.data.size = strlen(coordinates_msg.data.data);
    rcl_publish(&publisher_coordinates, &coordinates_msg, NULL);

    lastCoordinatePublishTime = millis();
  }
}
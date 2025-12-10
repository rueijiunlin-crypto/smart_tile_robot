#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <string.h>
#include "math.h"



// ==========================================
// 全局 ROS 變數定義
// ==========================================
rcl_subscription_t subscriber_led;
rcl_subscription_t subscriber_move;
rcl_subscription_t subscriber_keyboard_input;
rcl_publisher_t publisher_locate;       // 發布位置消息
rcl_publisher_t publisher_coordinates;  // 定時發布座標
std_msgs__msg__String locate_msg;
std_msgs__msg__String coordinates_msg;
rcl_subscription_t subscriber_move_hit_initialized;
std_msgs__msg__String move_hit_initialized_msg;

std_msgs__msg__Bool led_msg;
std_msgs__msg__String move_msg;
std_msgs__msg__String keyboard_input_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ==========================================
// 硬體針腳定義
// ==========================================
const int xstepPin = 16;
const int xdirPin = 17;
const int ystepPin = 26;
const int ydirPin = 25;
#define LED_PIN 2

// ==========================================
// 運動參數與幾何定義 (新算法核心)
// ==========================================
static unsigned long lastMoveTime = 0;
const unsigned long moveInterval = 100; // 移動間隔 (ms)

// 步進電機參數
const float steps_per_rotation = 200 * 16; // 1/16 微步
const float ropeLength_of_circle = 7.5;    // 捲揚圓周 (cm)
// 速度/脈衝安全限制
const unsigned long MIN_PULSE_US = 300;    // 最小脈衝寬度(含高低各一半)避免過快
const int SPIN_EVERY_STEPS = 20;           // 每 N 步讓 executor 處理一次回呼

// [新算法參數] 依據您的圖片設定
const float W = 92.0;  // 兩捲揚機總寬度 (cm)
const float h = 30.0;  // 初始垂掛高度 (cm)

// 修正係數 (預設 1.0)
const float x_correction_factor = 1.0; 

// [座標變數]
// 0,0 代表機器人位於 "兩捲揚機中間" 且 "高度為 h" 的位置
int current_x_disp = 0; // X 軸位移 (負左, 正右)
int current_y_disp = 0; // Y 軸位移 (正下)

// 記錄當前的繩長，用於計算變化量 (Delta)
float old_L1 = 0.0;
float old_L2 = 0.0;

// ==========================================
// 輔助函數：幾何計算與邊界檢查
// ==========================================

// 檢查座標是否在安全範圍內
bool check_coords_matrix(int x_disp, int y_disp) {
  // 假設 X 軸左右各允許移動 35cm (總寬92，預留邊距)
  // 假設 Y 軸只允許往下移動 50cm，不允許往上超過初始點 (y<0)
  bool x_safe = (x_disp >= -35 && x_disp <= 35);
  bool y_safe = (y_disp >= 0 && y_disp <= 50);
  return x_safe && y_safe;
}

// [新公式 (2)] 計算左繩 L1 長度
float calculate_L1_len(int x_disp, int y_disp) {
  float term_x = (W / 2.0) + (float)x_disp;
  float term_y = h + (float)y_disp;
  return sqrt(pow(term_x, 2) + pow(term_y, 2));
}

// [新公式 (3)] 計算右繩 L2 長度
float calculate_L2_len(int x_disp, int y_disp) {
  float term_x = (W / 2.0) - (float)x_disp; // 注意這裡是減號
  float term_y = h + (float)y_disp;
  return sqrt(pow(term_x, 2) + pow(term_y, 2));
}

// 發送單一脈衝（已加入最小脈衝時間保護）
void sendSinglePulse(int stepPin, unsigned long pulseDelay) {
  unsigned long d = (pulseDelay > MIN_PULSE_US) ? pulseDelay : MIN_PULSE_US;
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(d / 2);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(d / 2);
}

// ==========================================
// 馬達控制核心
// ==========================================
void setTargetPositions(float new_L1, float new_L2) {
  // 1. 計算繩長變化量
  float delta_L1 = new_L1 - old_L1;
  float delta_L2 = new_L2 - old_L2;

  // 2. 計算步數（含倍率），避免過大過快
  int steps_x = abs(delta_L1 * x_correction_factor * steps_per_rotation / ropeLength_of_circle) * 10;
  int steps_y = abs(delta_L2 * steps_per_rotation / ropeLength_of_circle) * 10;

  // 3. 計算延遲 (線性插補) 並套用最小脈衝限制
  unsigned long pulseDelay_x = (steps_x > 0) ? (500000 / steps_x) : 0;
  unsigned long pulseDelay_y = (steps_y > 0) ? (500000 / steps_y) : 0;
  if (pulseDelay_x < MIN_PULSE_US) pulseDelay_x = MIN_PULSE_US;
  if (pulseDelay_y < MIN_PULSE_US) pulseDelay_y = MIN_PULSE_US;

  // 4. 設置方向
  // 左邊(L1): >=0 為正轉(放繩)
  // 右邊(L2): <=0 為正轉(放繩) [這是依照您的特殊硬體邏輯]
  bool direction_x = delta_L1 >= 0; 
  bool direction_y = delta_L2 <= 0; 

  digitalWrite(xdirPin, direction_x);
  digitalWrite(ydirPin, direction_y);

  // 5. 同步驅動兩個馬達
  int max_steps = max(steps_x, steps_y);

  // 5. 同步驅動兩個馬達，並定期讓 executor 處理回呼（避免阻塞）
  for (int i = 0; i < max_steps; i++) {
    if (i < steps_x) {
      sendSinglePulse(xstepPin, pulseDelay_x);
    }
    if (i < steps_y) {
      sendSinglePulse(ystepPin, pulseDelay_y);
    }
    // 定期釋出 CPU 給 rclc executor
    if ((i % SPIN_EVERY_STEPS) == 0) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }
  }

  // 6. 更新舊繩長記錄
  old_L1 = new_L1;
  old_L2 = new_L2;
}

// ==========================================
// [核心重構] 統一處理移動邏輯的函式
// ==========================================
bool executeMoveLogic(char command) {
  int next_x = current_x_disp;
  int next_y = current_y_disp;

  // 1. 解析指令
  if (command == 'w' || command == 't') next_y -= 1;
  else if (command == 's' || command == 'g') next_y += 1;
  else if (command == 'a') next_x -= 1;
  else if (command == 'd') next_x += 1;
  else return false; // 無效指令

  // 2. 檢查邊界
  if (check_coords_matrix(next_x, next_y)) {
    // 3. 更新全域變數
    current_x_disp = next_x;
    current_y_disp = next_y;
    
    // 4. 計算新繩長 (新算法)
    float new_L1 = calculate_L1_len(current_x_disp, current_y_disp);
    float new_L2 = calculate_L2_len(current_x_disp, current_y_disp);
    
    // 5. 執行馬達移動
    setTargetPositions(new_L1, new_L2);

    // 6. 發布位置
    snprintf(locate_msg.data.data, 100, "Located at (%d, %d)", current_x_disp, current_y_disp);
    locate_msg.data.size = strlen(locate_msg.data.data);
    rcl_publish(&publisher_locate, &locate_msg, NULL);

    Serial.print("Moved to: ");
    Serial.print(current_x_disp);
    Serial.print(", ");
    Serial.println(current_y_disp);

    return true; // 移動成功
  } else {
    Serial.println("輸入的座標超出活動空間");
    return false; // 移動失敗
  }
}

// ==========================================
// 命令佇列 (Command Queue)
// ==========================================
#define COMMAND_QUEUE_SIZE 100
char commandQueue[COMMAND_QUEUE_SIZE];
int commandQueueHead = 0;
int commandQueueTail = 0;

bool isQueueEmpty() {
  return commandQueueHead == commandQueueTail;
}

bool isQueueFull() {
  return ((commandQueueTail + 1) % COMMAND_QUEUE_SIZE) == commandQueueHead;
}

bool enqueueCommand(char command) {
  if (!isQueueFull()) {
    commandQueue[commandQueueTail] = command;
    commandQueueTail = (commandQueueTail + 1) % COMMAND_QUEUE_SIZE;
    return true;
  }
  return false;
}

bool dequeueCommand(char *command) {
  if (!isQueueEmpty()) {
    *command = commandQueue[commandQueueHead];
    commandQueueHead = (commandQueueHead + 1) % COMMAND_QUEUE_SIZE;
    return true;
  }
  return false;
}

// ==========================================
// ROS 回調函數 (Callbacks)
// ==========================================

void subscription_led_callback(const void *msg_in) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
  if (msg->data) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
  }
}

// 單次移動回調 (/move2)
void subscription_move_callback(const void *msg_in) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;

  if (millis() - lastMoveTime > moveInterval) {
    if (msg->data.size > 0) {
       char cmd = msg->data.data[0];
       // 直接呼叫統一邏輯
       if (executeMoveLogic(cmd)) {
         lastMoveTime = millis();
       }
    }
  }
}

// 鍵盤輸入回調 (佇列處理)
void subscription_keyboard_input_callback(const void *msg_in) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
  if (msg->data.size > 0) {
    if (msg->data.size < 100) {
      msg->data.data[msg->data.size] = '\0';
    }
    Serial.print("Received command string: ");
    Serial.println(msg->data.data);

    for (size_t i = 0; i < msg->data.size; i++) {
      char command = msg->data.data[i];
      if (!enqueueCommand(command)) {
        Serial.println("Command queue full.");
      }
    }
  }
}

// 特殊初始化移動回調 (手動調整用)
void subscription_move_hit_initialized_callback(const void *msg_in) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
  if (msg->data.size > 0) {
    msg->data.data[msg->data.size] = '\0';
    char command = msg->data.data[0];
    int steps = strlen(msg->data.data); // 這邊用字串長度當步數是一個特殊設計

    if (command == 'a' || command == 'd') {
      bool direction = (command == 'a') ? LOW : HIGH;
      digitalWrite(xdirPin, direction);
      for (int i = 0; i < steps; i++) {
        sendSinglePulse(xstepPin);
        delay(50);
      }
    }
  }
}

// ==========================================
// Setup
// ==========================================

void setup() {
  // 初始化針腳
  pinMode(xstepPin, OUTPUT);
  pinMode(xdirPin, OUTPUT);
  pinMode(ystepPin, OUTPUT);
  pinMode(ydirPin, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 開機閃燈提示
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);

  // === [新算法初始化] ===
  // 假設開機時機器人在正中間，高度為 h (30cm)
  float start_term_x = W / 2.0;
  float start_term_y = h;
  float initial_length = sqrt(pow(start_term_x, 2) + pow(start_term_y, 2));
  
  old_L1 = initial_length;
  old_L2 = initial_length;
  current_x_disp = 0;
  current_y_disp = 0;

  // === [請修改這裡!] ROS WiFi 設定 ===
  // 參數: SSID, 密碼, Agent電腦IP, Agent Port
  set_microros_wifi_transports("您的WiFi名稱", "您的WiFi密碼", "您的電腦IP", 2024);

  // 初始化 micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "move_node", "", &support);

  // 建立訂閱者與發布者
  rclc_subscription_init_default(&subscriber_led, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/ros2_led");
  rclc_subscription_init_default(&subscriber_move, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/move2");
  rclc_subscription_init_default(&subscriber_keyboard_input, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/keyboard_input");
  rclc_subscription_init_default(&subscriber_move_hit_initialized, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/move_hit_initialized");
  
  rclc_publisher_init_default(&publisher_locate, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/top_base_locate");
  rclc_publisher_init_default(&publisher_coordinates, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/top_base_Coordinates");

  // 內存分配
  locate_msg.data.data = (char*)malloc(100 * sizeof(char));
  locate_msg.data.size = 0;
  locate_msg.data.capacity = 100;
  
  coordinates_msg.data.data = (char*)malloc(100 * sizeof(char));
  coordinates_msg.data.size = 0;
  coordinates_msg.data.capacity = 100;

  move_msg.data.data = (char*)malloc(100 * sizeof(char));
  move_msg.data.size = 0;
  move_msg.data.capacity = 100;

  keyboard_input_msg.data.data = (char*)malloc(1000 * sizeof(char));
  keyboard_input_msg.data.size = 0;
  keyboard_input_msg.data.capacity = 1000;

  move_hit_initialized_msg.data.data = (char*)malloc(100 * sizeof(char));
  move_hit_initialized_msg.data.size = 0;
  move_hit_initialized_msg.data.capacity = 100;

  // 初始化 Executor
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber_led, &led_msg, &subscription_led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_move, &move_msg, &subscription_move_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_keyboard_input, &keyboard_input_msg, &subscription_keyboard_input_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_move_hit_initialized, &move_hit_initialized_msg, &subscription_move_hit_initialized_callback, ON_NEW_DATA);
}

// ==========================================
// Loop
// ==========================================

void loop() {
  // 1. 處理 ROS 訊息 (Check for new messages)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // 2. 處理佇列命令 (Queue Processing)
  static bool isProcessing = false;

  if (!isProcessing && !isQueueEmpty()) {
    char command;
    if (dequeueCommand(&command)) {
      isProcessing = true;
      Serial.print("Processing command: ");
      Serial.println(command);

      // === 呼叫統一的移動邏輯 ===
      executeMoveLogic(command);
      // =======================

      lastMoveTime = millis();

      // 移動間隔等待 (非阻塞 ROS)
      unsigned long processStartTime = millis();
      while (millis() - processStartTime < moveInterval) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      isProcessing = false;
    }
  }

  // 3. 定期發布座標 (Heartbeat) - 讓 ROS 知道我還活著
  static unsigned long lastCoordinatePublishTime = 0;
  if (millis() - lastCoordinatePublishTime >= 1000) {
    snprintf(coordinates_msg.data.data, 100, "Current coordinates: (%d, %d)", current_x_disp, current_y_disp);
    coordinates_msg.data.size = strlen(coordinates_msg.data.data);
    rcl_publish(&publisher_coordinates, &coordinates_msg, NULL);
    lastCoordinatePublishTime = millis();
  }
}
#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>

// ===================== ROS 2 相關物件定義 =====================
// 建立訂閱者/發布者物件與訊息緩衝，讓 ESP32 能透過 micro-ROS 與 ROS2 通訊
rcl_subscription_t subscriber_led;                 
rcl_subscription_t subscriber_coordinates;         
std_msgs__msg__Bool   led_msg;                     
std_msgs__msg__String coordinates_msg;             
std_msgs__msg__String top_base_locate_msg;         
rcl_subscription_t    subscriber_top_base_locate;  
rcl_subscription_t    subscriber_keyboard;         
std_msgs__msg__String keyboard_msg;
rcl_subscription_t    subscriber_config;           // 配置訂閱者
std_msgs__msg__String config_msg;                  // 配置訊息                

rclc_executor_t executor;               // micro-ROS 執行器，用來執行 callback
rclc_support_t  support;                
rcl_allocator_t allocator;              
rcl_node_t      node;                   

rcl_publisher_t publisher_locate;                 // 發布到 /moveknock_locate_node → 控制錄音
std_msgs__msg__String locate_msg;                 
rcl_publisher_t publisher_move_hit_initialized;   // 發布到 /move_hit_initialized → 回報初始化完成
std_msgs__msg__String move_hit_initialized_msg;   

// ===================== 硬體腳位與狀態變數 =====================
#define LED_PIN 2
bool         base_locate_triggered = false;
unsigned long base_locate_timestamp = 0;

// ===================== 系統狀態監控 =====================
enum SystemState {
  STATE_INITIALIZING,
  STATE_READY,
  STATE_MOVING,
  STATE_HITTING,
  STATE_ERROR
};
SystemState current_state = STATE_INITIALIZING;
SystemState previous_state = STATE_INITIALIZING;
unsigned long last_heartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000; // 1秒心跳

// 狀態轉換保護
bool state_transition_locked = false;
unsigned long state_change_time = 0;
const unsigned long STATE_CHANGE_TIMEOUT = 5000; // 5秒狀態轉換超時

// 指令佇列管理
struct Command {
  char type;           // 'm' = move, 'h' = hit, 'i' = init
  float x, y;          // 移動目標座標
  unsigned long timestamp;
  bool processed;
};
Command command_queue[5];  // 最多5個指令佇列
int queue_head = 0;
int queue_tail = 0;
int queue_count = 0;
const unsigned long COMMAND_TIMEOUT = 10000; // 指令超時10秒

// ===================== 敲擊序列追蹤 =====================
int tile_index = 1;           // 磁磚編號（從 1 開始）
int hit_position = 0;         // 敲擊位置：0=left, 1=mid, 2=right
bool hit_sequence_active = false; // 是否在敲擊序列中

// ===================== 錯誤日誌管理 =====================
String last_error_message = "";
unsigned long last_error_time = 0;

// ===================== 非阻塞式移動控制 =====================
enum MoveState {
  MOVE_IDLE,
  MOVE_X_STEPPING,
  MOVE_Y_STEPPING,
  MOVE_COMPLETE
};
MoveState move_state = MOVE_IDLE;
int remaining_x_steps = 0;
int remaining_y_steps = 0;
unsigned long last_step_time = 0;
bool x_direction = true;
bool y_direction = true;

// ===================== 步進馬達控制腳位定義 =====================
const int xenablePin = 18;
const int yenablePin = 27;
const int xstepPin   = 16;
const int xdirPin    = 17;
const int ystepPin   = 26;
const int ydirPin    = 25;
const int xrightPin  = 33;   // X 軸右端限位（LOW 觸發）
const int ydownPin   = 32;   // Y 軸下端限位（LOW 觸發）

// ===================== 步進馬達參數設定 =====================
// 預設值（如果沒有收到配置則使用這些值）
float steps_per_rotation = 200.0f * 16.0f;  // 每圈 200 步，細分 1/16 → 3200 steps
float hit_of_circle      = 14.0f;           // 每圈絲桿/皮帶移動距離 (mm)
float max_x              = 350.0f;          // X 軸最大行程 (mm)
float max_y              = 140.0f;           // Y 軸最大行程 (mm)
float homing_speed_delay = 500;             // Homing 速度延遲 (微秒)
float step_delay         = 300;             // 步進延遲 (微秒) - 初始化預設為 300（越小越快）
unsigned long hit_timeout = 5000;           // 敲擊超時時間 (毫秒)

// 配置接收狀態
bool config_received = false;

// ===================== 當前位置座標 =====================
// 軟體座標（單位：mm），上電後透過 homing 校正
float current_x = 0.0f;
float current_y = 0.0f;

// ===================== 敲擊機構相關腳位與狀態 =====================
const int encoderPinB = 4;   // 預留（未使用編碼器）
const int motorPin1   = 21;  // 敲擊馬達控制腳 1
const int motorPin2   = 22;  // 敲擊馬達控制腳 2
const int limitSwitch = 13;  // 敲擊限位開關（LOW 觸發）
bool motorRunning     = false;

// ===================== 前置宣告 =====================
void moveToTarget(float target_x, float target_y);
void hitOnce();
void initializeAxes();
void cleanup_memory();
bool allocate_memory();
void deallocate_memory();
bool change_state(SystemState new_state, const char* reason = "");
void check_state_timeout();
String state_to_string(SystemState state);
void start_non_blocking_move(float target_x, float target_y);
void process_non_blocking_move();
void emergency_stop_move();
void log_error(const char* error_type, const char* details);
void publish_locate_text(const char* txt);


// 指令佇列管理函數
bool enqueue_command(char type, float x = 0.0f, float y = 0.0f);
bool dequeue_command();
void process_command_queue();
void clear_command_queue();
bool is_command_queue_full();
bool is_command_queue_empty();

// ===================== 步進馬達控制函數 =====================
// 送一個脈衝給 stepPin → 馬達走一步
static inline void pulseStep(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(step_delay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(step_delay);
}

// 送一步，包含方向設定
void stepMotor(int stepPin, int dirPin, bool direction) {
  digitalWrite(dirPin, direction ? HIGH : LOW);
  pulseStep(stepPin);
}

// ===================== Homing 功能 =====================
// 執行單軸 homing
// stepPin / dirPin → 該軸的步進訊號與方向腳位
// limitPin → 該軸的限位開關
// directionPositive → 往正方向 (HIGH) or 負方向 (LOW)
// axis_pos → 更新該軸的座標變數
// home_value → homing 完成後要設定的座標值
void homeAxis(int stepPin, int dirPin, int limitPin, bool directionPositive, float &axis_pos, float home_value) {
  digitalWrite(dirPin, directionPositive ? HIGH : LOW);

  // 加入超時保護，避免無限迴圈
  unsigned long homing_start = millis();
  const unsigned long HOMING_TIMEOUT = 30000; // 30秒超時
  int step_count = 0;
  const int MAX_STEPS = 50000; // 最大步數限制

  // 不斷送 step，直到觸發限位
  while (digitalRead(limitPin) == HIGH) {
    // 檢查超時
    if (millis() - homing_start > HOMING_TIMEOUT) {
      String error_details = "Axis: " + String((stepPin == xstepPin) ? "X" : "Y") + 
                            ", Steps: " + String(step_count) + 
                            ", Timeout: " + String(HOMING_TIMEOUT) + "ms";
      log_error("HOMING_TIMEOUT", error_details.c_str());
      change_state(STATE_ERROR, "Homing timeout");
      return;
    }
    
    // 檢查最大步數
    if (step_count > MAX_STEPS) {
      String error_details = "Axis: " + String((stepPin == xstepPin) ? "X" : "Y") + 
                            ", Steps: " + String(step_count) + 
                            ", Max: " + String(MAX_STEPS);
      log_error("HOMING_MAX_STEPS", error_details.c_str());
      change_state(STATE_ERROR, "Homing max steps exceeded");
      return;
    }
    
    pulseStep(stepPin);
    step_count++;
    delayMicroseconds(homing_speed_delay); // 控制 homing 速度
  }

  // 退回5mm避免壓著開關（移動機構）
  float retreat_distance = 5.0f; // 5mm
  int retreat_steps = (int)(retreat_distance * (steps_per_rotation / hit_of_circle));
  
  Serial.printf("Retreating %d steps (%.1f mm) from limit switch\n", retreat_steps, retreat_distance);
  
  // 反轉方向後退回
  digitalWrite(dirPin, !directionPositive);
  for (int i = 0; i < retreat_steps; i++) {
    pulseStep(stepPin);
    delayMicroseconds(homing_speed_delay);
  }

  // 更新軟體座標
  axis_pos = home_value;
}

// 執行整個 XY 軸 homing
void initializeAxes() {
  Serial.println("Starting simultaneous X/Y axis homing...");
  
  bool xHomed = false;
  bool yHomed = false;
  
  // 設定方向：X軸向右，Y軸向下
  digitalWrite(xdirPin, LOW);   // X軸向右移動
  digitalWrite(ydirPin, LOW);   // Y軸向下移動
  
  unsigned long homing_start = millis();
  const unsigned long HOMING_TIMEOUT = 30000; // 30秒超時
  int step_count = 0;
  const int MAX_STEPS = 50000; // 最大步數限制
  
  while (!xHomed || !yHomed) {
    // 超時檢查
    if (millis() - homing_start > HOMING_TIMEOUT) {
      String error_details = "Simultaneous homing timeout, Steps: " + String(step_count) + 
                            ", Timeout: " + String(HOMING_TIMEOUT) + "ms";
      log_error("SIMULTANEOUS_HOMING_TIMEOUT", error_details.c_str());
      change_state(STATE_ERROR, "Homing timeout");
      Serial.println("Error: Homing timeout!");
      return;
    }
    
    // 步數限制檢查
    if (step_count > MAX_STEPS) {
      String error_details = "Simultaneous homing max steps exceeded, Steps: " + String(step_count) + 
                            ", Max: " + String(MAX_STEPS);
      log_error("SIMULTANEOUS_HOMING_MAX_STEPS", error_details.c_str());
      change_state(STATE_ERROR, "Homing max steps exceeded");
      Serial.println("Error: Homing max steps exceeded!");
      return;
    }
    
    // 同時步進兩個軸（如果還沒歸位）
    if (!xHomed) {
      pulseStep(xstepPin);
    }
    if (!yHomed) {
      pulseStep(ystepPin);
    }
    
    // 檢查限位開關狀態（HIGH 觸發，N.O. 常開）
    if (digitalRead(xrightPin) == HIGH) {
      xHomed = true;
      current_x = max_x;  // 右限位 = MAX_X
      Serial.println("X-axis homed");
    }
    
    if (digitalRead(ydownPin) == HIGH) {
      yHomed = true;
      current_y = 0.0f;   // 下限位 = 0
      Serial.println("Y-axis homed");
    }
    
    step_count++;
    delayMicroseconds(homing_speed_delay);
  }
  
  Serial.println("Both axes homed successfully");
  
  // 退回5mm避免壓著限位開關（移動機構）
  float retreat_distance = 5.0f; // 5mm
  int retreat_x_steps = (int)(retreat_distance * (steps_per_rotation / hit_of_circle));
  int retreat_y_steps = (int)(retreat_distance * (steps_per_rotation / hit_of_circle));
  
  Serial.printf("Retreating X: %d steps (%.1f mm), Y: %d steps (%.1f mm)\n", 
               retreat_x_steps, retreat_distance, retreat_y_steps, retreat_distance);
  
  // X軸退回（向左移動）
  digitalWrite(xdirPin, HIGH); // 向左移動
  for (int i = 0; i < retreat_x_steps; i++) {
    pulseStep(xstepPin);
    delayMicroseconds(homing_speed_delay);
  }
  
  // Y軸退回（向上移動）
  digitalWrite(ydirPin, HIGH); // 向上移動
  for (int i = 0; i < retreat_y_steps; i++) {
    pulseStep(ystepPin);
    delayMicroseconds(homing_speed_delay);
  }
  
  // 更新最終座標
  current_x = max_x - retreat_distance;
  current_y = retreat_distance;
  
  Serial.printf("Final position after retreat: X=%.2f, Y=%.2f\n", current_x, current_y);
}

// ===================== 精確限位開關檢查函數（適合兩顆限位開關） =====================
bool checkLimitSwitches(bool x_direction, bool y_direction) {
  // 檢查 X 軸限位（只有右限位開關）
  if (x_direction && digitalRead(xrightPin) == HIGH) {
    Serial.println("X-axis hit right limit - stopping movement, can move left");
    return false; // 向右移動且碰到右限位 - 停止移動，允許向左移動
  }
  // 注意：沒有左限位開關，所以向左移動時無法檢查左限位
  
  // 檢查 Y 軸限位（只有下限位開關）
  if (y_direction && digitalRead(ydownPin) == HIGH) {
    Serial.println("Y-axis hit down limit - stopping movement, can move up");
    return false; // 向下移動且碰到下限位 - 停止移動，允許向上移動
  }
  // 注意：沒有上限位開關，所以向上移動時無法檢查上限位
  
  return true; // 所有檢查通過
}

// ===================== 非阻塞式移動控制函數 =====================
void start_non_blocking_move(float target_x, float target_y) {
  // 範圍檢查，避免撞機
  if (target_x < 0.0f || target_x > max_x || target_y < 0.0f || target_y > max_y) {
    Serial.printf("Warning: Target coordinates out of range! X=%.2f (max=%.1f), Y=%.2f (max=%.1f)\n", 
                  target_x, max_x, target_y, max_y);
    
    // 發布超過距離訊息
    String out_of_range_msg = "Out of range: X=" + String(target_x, 2) + " (max=" + String(max_x, 1) + 
                              "), Y=" + String(target_y, 2) + " (max=" + String(max_y, 1) + ")";
    publish_locate_text(out_of_range_msg.c_str());
    
    // 停止移動但保持 READY 狀態
    change_state(STATE_READY, "Target out of range - movement stopped");
    Serial.println("Movement stopped due to out of range coordinates");
    return;
  }

  // 檢查是否正在執行其他動作
  if (current_state == STATE_HITTING) {
    Serial.println("Warning: Currently hitting, cannot move!"); // 警告：正在敲擊中，無法移動！
    return;
  }

  if (!change_state(STATE_MOVING, "Move command received")) {
    Serial.println("Failed to change state to MOVING");
    return;
  }

  // 換算成步數
  float x_steps_f = (target_x - current_x) * (steps_per_rotation / hit_of_circle);
  float y_steps_f = (target_y - current_y) * (steps_per_rotation / hit_of_circle);
  remaining_x_steps = (int)(x_steps_f >= 0 ? floorf(x_steps_f + 0.5f) : ceilf(x_steps_f - 0.5f));
  remaining_y_steps = (int)(y_steps_f >= 0 ? floorf(y_steps_f + 0.5f) : ceilf(y_steps_f - 0.5f));
  
  x_direction = (remaining_x_steps >= 0);
  y_direction = (remaining_y_steps >= 0);
  
  remaining_x_steps = abs(remaining_x_steps);
  remaining_y_steps = abs(remaining_y_steps);

  // 開始非阻塞式移動
  move_state = MOVE_X_STEPPING;
  last_step_time = millis();
  
  Serial.printf("Starting non-blocking move to: (%.2f, %.2f), X_steps: %d, Y_steps: %d\n", 
               target_x, target_y, remaining_x_steps, remaining_y_steps);
}

void process_non_blocking_move() {
  if (move_state == MOVE_IDLE) return;
  
  // 檢查是否到了執行下一步的時間
  if (millis() - last_step_time < step_delay / 1000) return;
  
  if (move_state == MOVE_X_STEPPING) {
    if (remaining_x_steps > 0) {
      // 使用精確的限位檢查
      if (!checkLimitSwitches(x_direction, true)) { // Y軸暫時設為true，只檢查X軸
        emergency_stop_move();
        change_state(STATE_READY, "X-axis limit reached - can move left");
        Serial.println("X-axis movement stopped at limit, ready for reverse movement");
        return;
      }
      
      // 執行一步
      stepMotor(xstepPin, xdirPin, x_direction);
      remaining_x_steps--;
      last_step_time = millis();
      
      // 更新當前座標
      float step_distance = hit_of_circle / steps_per_rotation;
      current_x += (x_direction ? step_distance : -step_distance);
      
      // 每 100 步回報一次進度
      if (remaining_x_steps % 100 == 0) {
        Serial.printf("X-axis progress: %d steps remaining, current_x: %.2f\n", remaining_x_steps, current_x);
      }
    } else {
      // X 軸移動完成，開始 Y 軸
      move_state = MOVE_Y_STEPPING;
      Serial.println("X-axis move completed, starting Y-axis");
    }
  }
  
  if (move_state == MOVE_Y_STEPPING) {
    if (remaining_y_steps > 0) {
      // 使用精確的限位檢查
      if (!checkLimitSwitches(true, y_direction)) { // X軸暫時設為true，只檢查Y軸
        emergency_stop_move();
        change_state(STATE_READY, "Y-axis limit reached - can move up");
        Serial.println("Y-axis movement stopped at limit, ready for reverse movement");
        return;
      }
      
      // 執行一步
      stepMotor(ystepPin, ydirPin, y_direction);
      remaining_y_steps--;
      last_step_time = millis();
      
      // 更新當前座標
      float step_distance = hit_of_circle / steps_per_rotation;
      current_y += (y_direction ? step_distance : -step_distance);
      
      // 每 100 步回報一次進度
      if (remaining_y_steps % 100 == 0) {
        Serial.printf("Y-axis progress: %d steps remaining, current_y: %.2f\n", remaining_y_steps, current_y);
      }
    } else {
      // Y 軸移動完成
      move_state = MOVE_COMPLETE;
      Serial.println("Y-axis move completed");
    }
  }
  
  if (move_state == MOVE_COMPLETE) {
    // 移動完成，座標已在移動過程中更新
    move_state = MOVE_IDLE;
    change_state(STATE_READY, "Move completed");
    Serial.printf("Non-blocking move completed, current coordinates: (%.2f, %.2f)\n", current_x, current_y);
    
    // 發布當前位置
    publish_position();
  }
}

void emergency_stop_move() {
  move_state = MOVE_IDLE;
  remaining_x_steps = 0;
  remaining_y_steps = 0;
  
  // 不斷電，只停止移動（馬達保持啟用狀態）
  // digitalWrite(xenablePin, HIGH);  // 註解掉，不斷電
  // digitalWrite(yenablePin, HIGH);  // 註解掉，不斷電
  
  Serial.println("Emergency stop: Movement halted (motors remain enabled)");
}

// 緊急停止所有馬達（包括敲擊馬達）
void emergency_stop_all_motors() {
  // 停止移動馬達（不斷電）
  emergency_stop_move();
  
  // 停止敲擊馬達
  if (motorRunning) {
    stopMotor();
    motorRunning = false;
    Serial.println("Emergency stop: Hit motor stopped");
  }
  
  // 清空指令佇列
  clear_command_queue();
  
  Serial.println("Emergency stop: Movement stopped, motors remain enabled");
}

// ===================== 移動到目標位置（保持向後相容） =====================
void moveToTarget(float target_x, float target_y) {
  start_non_blocking_move(target_x, target_y);
}

// ===================== 敲擊馬達控制 =====================
// 啟動敲擊馬達
void startMotor() {
  Serial.println("DEBUG: Starting motor - Pin1=HIGH, Pin2=LOW");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  Serial.printf("DEBUG: Motor pins set - Pin1=%d, Pin2=%d\n", digitalRead(motorPin1), digitalRead(motorPin2));
}

// 停止敲擊馬達
void stopMotor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

// ===================== 小工具：發佈文字到 /moveknock_locate_node =====================
void publish_locate_text(const char* txt) {
  size_t n = strlen(txt);
  if (n >= locate_msg.data.capacity) n = locate_msg.data.capacity - 1;
  memcpy(locate_msg.data.data, txt, n);
  locate_msg.data.data[n] = '\0';
  locate_msg.data.size = n;
  rcl_publish(&publisher_locate, &locate_msg, NULL);
}

// ===================== 發布當前位置到 /moveknock_locate_node =====================
void publish_position() {
  String position_str = "Current position: X:" + String(current_x, 2) + ", Y:" + String(current_y, 2);
  publish_locate_text(position_str.c_str());
  Serial.println("Published position: " + position_str);
}

// ===================== 上線回報 OK 到 /move_hit_initialized =====================
void publish_initialized_ok() {
  const char* ok = "OK";
  size_t n = 2;
  memcpy(move_hit_initialized_msg.data.data, ok, n);
  move_hit_initialized_msg.data.data[n] = '\0';
  move_hit_initialized_msg.data.size = n;
  rcl_publish(&publisher_move_hit_initialized, &move_hit_initialized_msg, NULL);
}

// ===================== ROS 2 訂閱者回調函數 =====================
// 控制 LED
void subscription_led_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  digitalWrite(LED_PIN, msg->data);
}

// ===================== 處理多個座標字串 =====================
void processCoordinateString(String coord_str) {
  // 清理座標字串格式
  coord_str.replace("),(", "|");  // 用 | 分隔座標
  coord_str.replace("(", "");     // 移除左括號
  coord_str.replace(")", "");     // 移除右括號
  coord_str.trim();               // 移除前後空白
  
  Serial.println("Processing coordinates: " + coord_str);
  
  int startIndex = 0;
  int endIndex = coord_str.indexOf('|');
  int coord_count = 0;
  
  // 處理所有座標
  while (endIndex != -1) {
    String coord = coord_str.substring(startIndex, endIndex);
    if (processSingleCoordinate(coord)) {
      coord_count++;
    }
    startIndex = endIndex + 1;
    endIndex = coord_str.indexOf('|', startIndex);
  }
  
  // 處理最後一個座標
  if (startIndex < coord_str.length()) {
    String coord = coord_str.substring(startIndex);
    if (processSingleCoordinate(coord)) {
      coord_count++;
    }
  }
  
  Serial.printf("Processed %d coordinates successfully\n", coord_count);
  
  // 完成後回到原點
  if (coord_count > 0) {
    Serial.println("Returning to origin...");
    moveToTarget(0.0f, 0.0f);
  }
}

// ===================== 處理單個座標 =====================
bool processSingleCoordinate(String coord) {
  int separatorIndex = coord.indexOf(',');
  
  if (separatorIndex == -1) {
    Serial.println("Error: Invalid coordinate format - missing comma");
    return false;
  }
  
  float x = coord.substring(0, separatorIndex).toFloat();
  float y = coord.substring(separatorIndex + 1).toFloat();
  
  // 檢查座標範圍
  if (x < 0.0f || x > max_x || y < 0.0f || y > max_y) {
    Serial.printf("Warning: Coordinates out of range! X=%.2f (max=%.1f), Y=%.2f (max=%.1f)\n", 
                  x, max_x, y, max_y);
    
    // 發布超過距離訊息
    String out_of_range_msg = "Out of range: X=" + String(x, 2) + " (max=" + String(max_x, 1) + 
                              "), Y=" + String(y, 2) + " (max=" + String(max_y, 1) + ")";
    publish_locate_text(out_of_range_msg.c_str());
    
    // 停止處理但保持 READY 狀態
    change_state(STATE_READY, "Coordinates out of range - processing stopped");
    Serial.println("Coordinate processing stopped due to out of range");
    return false;
  }
  
  Serial.printf("Moving to coordinate: (%.2f, %.2f)\n", x, y);
  
  // 移動到目標位置
  moveToTarget(x, y);
  
  // 發布位置信息
  String locate_msg_str = "Target located at X: " + String(x, 2) + ", Y: " + String(y, 2);
  publish_locate_text(locate_msg_str.c_str());
  
  // 執行敲擊
  hitOnce();
  
  return true;
}

// ===================== ROS 2 訂閱者回調函數 =====================
// 接收 "x,y" 座標 → 移動到該點（改進版）
void subscription_coordinates_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  // 檢查是否有觸發標誌
  if (!base_locate_triggered || (millis() - base_locate_timestamp > 1500)) {
    Serial.println("Ignoring /World_Coordinates as no recent /top_base_locate was received.");
    return;
  }
  
  Serial.println("Received coordinates: " + String(msg->data.data));
  
  // 處理座標字串（支援多個座標）
  processCoordinateString(String(msg->data.data));
  
  // 清理觸發標誌
  base_locate_triggered = false;
}

// 接收 /top_base_locate → 記錄觸發時間
void subscription_top_base_locate_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (strcmp(msg->data.data, "Start") == 0) {
    base_locate_triggered = true;
    base_locate_timestamp = millis();
  }
}

// 接收 /keyboard_control 指令
void subscription_keyboard_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (msg->data.size < 1) return;
  char key = msg->data.data[0];
  
  switch (key) {
    case 'w': enqueue_command('m', current_x, current_y + 10.0f); break; // Y+10
    case 's': enqueue_command('m', current_x, current_y - 10.0f); break; // Y-10
    case 'a': enqueue_command('m', current_x - 10.0f, current_y); break; // X-10
    case 'd': enqueue_command('m', current_x + 10.0f, current_y); break; // X+10
    case 'h': enqueue_command('h'); break;                     // 敲擊一次
    case 'r': enqueue_command('m', 0.0f, 0.0f); break;         // 回到 (0,0)
    case 'p': publish_position(); break;                       // 查詢當前位置
    case 'e': 
      if (last_error_message.length() > 0) {
        String error_report = "Last error: " + last_error_message + 
                             " (Time: " + String(last_error_time) + "ms ago)";
        publish_locate_text(error_report.c_str());
        Serial.println(error_report);
      } else {
        publish_locate_text("No errors recorded");
        Serial.println("No errors recorded");
      }
      break;                                                    // 查詢最後錯誤
    case 'i': enqueue_command('i'); break;                      // 重新 homing
    case 'c': 
      // 清除錯誤狀態，回到 READY
      if (current_state == STATE_ERROR) {
        change_state(STATE_READY, "Error cleared by user");
        Serial.println("Error state cleared, system ready");
      }
      break;
    case 'm':
      // 馬達測試指令
      Serial.println("=== 馬達硬體測試 ===");
      Serial.printf("Motor Pin1 (GPIO %d): %s\n", motorPin1, digitalRead(motorPin1) ? "HIGH" : "LOW");
      Serial.printf("Motor Pin2 (GPIO %d): %s\n", motorPin2, digitalRead(motorPin2) ? "HIGH" : "LOW");
      Serial.printf("Limit Switch (GPIO %d): %s\n", limitSwitch, digitalRead(limitSwitch) ? "HIGH" : "LOW");
      Serial.printf("Motor Running: %s\n", motorRunning ? "YES" : "NO");
      
      // 測試馬達啟動
      Serial.println("Testing motor start...");
      startMotor();
      delay(2000);
      Serial.println("Testing motor stop...");
      stopMotor();
      Serial.println("Motor test completed");
      break;
    case 'M':
      // 移動馬達測試指令
      Serial.println("=== 移動馬達硬體測試 ===");
      Serial.printf("X軸步進馬達 - STEP: GPIO %d, DIR: GPIO %d, EN: GPIO %d\n", xstepPin, xdirPin, xenablePin);
      Serial.printf("Y軸步進馬達 - STEP: GPIO %d, DIR: GPIO %d, EN: GPIO %d\n", ystepPin, ydirPin, yenablePin);
      Serial.printf("X軸限位開關 (GPIO %d): %s\n", xrightPin, digitalRead(xrightPin) ? "HIGH" : "LOW");
      Serial.printf("Y軸限位開關 (GPIO %d): %s\n", ydownPin, digitalRead(ydownPin) ? "HIGH" : "LOW");
      Serial.printf("當前位置: X=%.2f, Y=%.2f\n", current_x, current_y);
      Serial.printf("移動狀態: %s\n", move_state == MOVE_IDLE ? "IDLE" : "MOVING");
      
      // 測試 X 軸移動
      Serial.println("Testing X-axis movement (10 steps)...");
      digitalWrite(xenablePin, LOW); // 啟用馬達
      digitalWrite(xdirPin, HIGH);   // 設定方向（向左）
      for (int i = 0; i < 10; i++) {
        digitalWrite(xstepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(xstepPin, LOW);
        delayMicroseconds(1000);
        Serial.printf("X step %d/10\n", i+1);
      }
      // digitalWrite(xenablePin, HIGH); // 註解掉，不斷電
      Serial.println("X-axis test completed (motor remains enabled)");
      
      delay(1000);
      
      // 測試 Y 軸移動
      Serial.println("Testing Y-axis movement (10 steps)...");
      digitalWrite(yenablePin, LOW); // 啟用馬達
      digitalWrite(ydirPin, HIGH);   // 設定方向（向上）
      for (int i = 0; i < 10; i++) {
        digitalWrite(ystepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(ystepPin, LOW);
        delayMicroseconds(1000);
        Serial.printf("Y step %d/10\n", i+1);
      }
      // digitalWrite(yenablePin, HIGH); // 註解掉，不斷電
      Serial.println("Y-axis test completed (motor remains enabled)");
      
      Serial.println("=== 移動馬達測試完成 ===");
      break;
    case 't': 
      // 啟動敲擊序列
      hit_sequence_active = true;
      hit_position = 0; // 從左邊開始
      Serial.println("Hit sequence started - left position");
      break;
    case 'n':
      // 下一個敲擊位置
      if (hit_sequence_active) {
        hit_position = (hit_position + 1) % 3;
        String positions[] = {"left", "mid", "right"};
        Serial.println("Next position: " + String(positions[hit_position]));
      }
      break;
    case 'q':
      // 結束敲擊序列
      hit_sequence_active = false;
      tile_index++;
      Serial.println("Hit sequence ended. Next tile: " + String(tile_index));
      break;
    case 'x':
      // 緊急停止
      emergency_stop_move();
      if (motorRunning) {
        stopMotor();
        motorRunning = false;
      }
      clear_command_queue(); // 清空指令佇列
      change_state(STATE_READY, "Emergency stop");
      Serial.println("Emergency stop executed");
      break;
    case '+':
      step_delay = max(100.0f, step_delay - 50.0f); // 加速（下限 100us）
      Serial.printf("Speed up: step_delay=%d us\n", (int)step_delay);
      break;
    case '-':
      step_delay = min(2000.0f, step_delay + 50.0f); // 減速（上限 2000us）
      Serial.printf("Slow down: step_delay=%d us\n", (int)step_delay);
      break;
    default: break;
  }
}

// 接收系統配置
void subscription_config_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  // 簡單的 JSON 解析（ESP32 版本）
  String config_str = String(msg->data.data);
  
  // 解析 max_x_travel
  int start = config_str.indexOf("\"max_x_travel\":");
  if (start != -1) {
    start = config_str.indexOf(":", start) + 1;
    int end = config_str.indexOf(",", start);
    if (end == -1) end = config_str.indexOf("}", start);
    max_x = config_str.substring(start, end).toFloat();
  }
  
  // 解析 max_y_travel
  start = config_str.indexOf("\"max_y_travel\":");
  if (start != -1) {
    start = config_str.indexOf(":", start) + 1;
    int end = config_str.indexOf(",", start);
    if (end == -1) end = config_str.indexOf("}", start);
    max_y = config_str.substring(start, end).toFloat();
  }
  
  // 解析 steps_per_rotation
  start = config_str.indexOf("\"steps_per_rotation\":");
  if (start != -1) {
    start = config_str.indexOf(":", start) + 1;
    int end = config_str.indexOf(",", start);
    if (end == -1) end = config_str.indexOf("}", start);
    steps_per_rotation = config_str.substring(start, end).toFloat();
  }
  
  // 解析 mm_per_rotation
  start = config_str.indexOf("\"mm_per_rotation\":");
  if (start != -1) {
    start = config_str.indexOf(":", start) + 1;
    int end = config_str.indexOf(",", start);
    if (end == -1) end = config_str.indexOf("}", start);
    hit_of_circle = config_str.substring(start, end).toFloat();
  }
  
  // 解析 homing_speed_delay
  start = config_str.indexOf("\"homing_speed_delay\":");
  if (start != -1) {
    start = config_str.indexOf(":", start) + 1;
    int end = config_str.indexOf(",", start);
    if (end == -1) end = config_str.indexOf("}", start);
    homing_speed_delay = config_str.substring(start, end).toInt();
  }
  
  // 解析 step_delay
  start = config_str.indexOf("\"step_delay\":");
  if (start != -1) {
    start = config_str.indexOf(":", start) + 1;
    int end = config_str.indexOf(",", start);
    if (end == -1) end = config_str.indexOf("}", start);
    step_delay = config_str.substring(start, end).toInt();
  }
  
  // 解析 hit_timeout
  start = config_str.indexOf("\"hit_timeout\":");
  if (start != -1) {
    start = config_str.indexOf(":", start) + 1;
    int end = config_str.indexOf(",", start);
    if (end == -1) end = config_str.indexOf("}", start);
    hit_timeout = config_str.substring(start, end).toInt();
  }
  
  config_received = true;
  Serial.printf("Configuration updated: max_x=%.1f, max_y=%.1f, steps_per_rotation=%.0f\n", 
                max_x, max_y, steps_per_rotation);
}

// ===================== 記憶體管理函數 =====================
bool allocate_memory() {
  // 分配記憶體並檢查是否成功
  coordinates_msg.data.data = (char *)malloc(100);
  if (!coordinates_msg.data.data) {
    Serial.println("Error: Failed to allocate memory for coordinates_msg");
    return false;
  }
  coordinates_msg.data.size = 0;
  coordinates_msg.data.capacity = 100;

  move_hit_initialized_msg.data.data = (char *)malloc(50);
  if (!move_hit_initialized_msg.data.data) {
    Serial.println("Error: Failed to allocate memory for move_hit_initialized_msg");
    deallocate_memory();
    return false;
  }
  move_hit_initialized_msg.data.size = 0;
  move_hit_initialized_msg.data.capacity = 50;

  top_base_locate_msg.data.data = (char *)malloc(100);
  if (!top_base_locate_msg.data.data) {
    Serial.println("Error: Failed to allocate memory for top_base_locate_msg");
    deallocate_memory();
    return false;
  }
  top_base_locate_msg.data.size = 0;
  top_base_locate_msg.data.capacity = 100;

  locate_msg.data.data = (char *)malloc(100);
  if (!locate_msg.data.data) {
    Serial.println("Error: Failed to allocate memory for locate_msg");
    deallocate_memory();
    return false;
  }
  locate_msg.data.size = 0;
  locate_msg.data.capacity = 100;

  keyboard_msg.data.data = (char *)malloc(10);
  if (!keyboard_msg.data.data) {
    Serial.println("Error: Failed to allocate memory for keyboard_msg");
    deallocate_memory();
    return false;
  }
  keyboard_msg.data.size = 0;
  keyboard_msg.data.capacity = 10;

  config_msg.data.data = (char *)malloc(500);
  if (!config_msg.data.data) {
    Serial.println("Error: Failed to allocate memory for config_msg");
    deallocate_memory();
    return false;
  }
  config_msg.data.size = 0;
  config_msg.data.capacity = 500;

  Serial.println("Memory allocation successful");
  return true;
}

void deallocate_memory() {
  // 安全釋放記憶體
  if (coordinates_msg.data.data) {
    free(coordinates_msg.data.data);
    coordinates_msg.data.data = NULL;
  }
  if (move_hit_initialized_msg.data.data) {
    free(move_hit_initialized_msg.data.data);
    move_hit_initialized_msg.data.data = NULL;
  }
  if (top_base_locate_msg.data.data) {
    free(top_base_locate_msg.data.data);
    top_base_locate_msg.data.data = NULL;
  }
  if (locate_msg.data.data) {
    free(locate_msg.data.data);
    locate_msg.data.data = NULL;
  }
  if (keyboard_msg.data.data) {
    free(keyboard_msg.data.data);
    keyboard_msg.data.data = NULL;
  }
  if (config_msg.data.data) {
    free(config_msg.data.data);
    config_msg.data.data = NULL;
  }
  Serial.println("Memory deallocated");
}

void cleanup_memory() {
  // 定期清理記憶體碎片
  static unsigned long last_cleanup = 0;
  const unsigned long CLEANUP_INTERVAL = 300000; // 5分鐘清理一次
  
  if (millis() - last_cleanup > CLEANUP_INTERVAL) {
    unsigned long free_heap = ESP.getFreeHeap();
    Serial.printf("Memory status - Free heap: %d bytes\n", free_heap);
    
    // 如果記憶體不足，嘗試清理
    if (free_heap < 10000) { // 少於 10KB
      Serial.println("Warning: Low memory detected!");
      // 可以考慮重啟系統或清理快取
    }
    
    last_cleanup = millis();
  }
}

// ===================== 指令佇列管理函數 =====================
bool enqueue_command(char type, float x, float y) {
  if (is_command_queue_full()) {
    Serial.println("Warning: Command queue is full, dropping command");
    return false;
  }
  
  command_queue[queue_tail].type = type;
  command_queue[queue_tail].x = x;
  command_queue[queue_tail].y = y;
  command_queue[queue_tail].timestamp = millis();
  command_queue[queue_tail].processed = false;
  
  queue_tail = (queue_tail + 1) % 5;
  queue_count++;
  
  Serial.printf("Command enqueued: type=%c, x=%.2f, y=%.2f, queue_count=%d\n", 
                type, x, y, queue_count);
  return true;
}

bool dequeue_command() {
  if (is_command_queue_empty()) {
    return false;
  }
  
  queue_head = (queue_head + 1) % 5;
  queue_count--;
  return true;
}

void process_command_queue() {
  if (is_command_queue_empty()) return;
  
  Command* cmd = &command_queue[queue_head];
  
  // 檢查指令超時
  if (millis() - cmd->timestamp > COMMAND_TIMEOUT) {
    Serial.printf("Command timeout: type=%c, age=%d ms\n", 
                  cmd->type, (int)(millis() - cmd->timestamp));
    dequeue_command();
    return;
  }
  
  // 檢查系統狀態是否允許執行指令
  if (current_state == STATE_ERROR) {
    Serial.println("System in error state, clearing command queue");
    clear_command_queue();
    return;
  }
  
  // 執行指令
  switch (cmd->type) {
    case 'm': // 移動指令
      if (current_state == STATE_READY) {
        start_non_blocking_move(cmd->x, cmd->y);
        cmd->processed = true;
        dequeue_command();
      }
      break;
      
    case 'h': // 敲擊指令
      if (current_state == STATE_READY) {
        hitOnce();
        cmd->processed = true;
        dequeue_command();
      }
      break;
      
    case 'i': // 初始化指令
      if (current_state == STATE_READY) {
        initializeAxes();
        cmd->processed = true;
        dequeue_command();
      }
      break;
      
    default:
      Serial.printf("Unknown command type: %c\n", cmd->type);
      dequeue_command();
      break;
  }
}

void clear_command_queue() {
  queue_head = 0;
  queue_tail = 0;
  queue_count = 0;
  Serial.println("Command queue cleared");
}

bool is_command_queue_full() {
  return queue_count >= 5;
}

bool is_command_queue_empty() {
  return queue_count == 0;
}


void log_error(const char* error_type, const char* details) {
  unsigned long current_time = millis();
  String timestamp = String(current_time);
  String error_log = "ERROR[" + timestamp + "]: " + String(error_type) + " - " + String(details);
  
  // 序列埠輸出
  Serial.println(error_log);
  
  // ROS2 話題回報
  String ros_error_msg = "error:" + String(error_type) + ":" + String(details);
  publish_locate_text(ros_error_msg.c_str());
  
  // 儲存錯誤訊息
  last_error_message = error_log;
  last_error_time = current_time;
  
  // 只停止敲擊馬達，移動馬達保持啟用
  if (motorRunning) {
    stopMotor();
    motorRunning = false;
    Serial.println("Error: Hit motor stopped, movement motors remain enabled");
  }
  
  // 清空指令佇列
  clear_command_queue();
}

// ===================== 狀態機管理函數 =====================
bool change_state(SystemState new_state, const char* reason) {
  // 檢查狀態轉換是否合法
  if (state_transition_locked && !(new_state == STATE_ERROR || new_state == STATE_READY)) {
    String error_details = "State transition blocked! Current: " + String(state_to_string(current_state).c_str()) + 
                          ", Requested: " + String(state_to_string(new_state).c_str());
    log_error("STATE_TRANSITION_BLOCKED", error_details.c_str());
    return false;
  }
  
  // 檢查狀態轉換邏輯
  bool valid_transition = false;
  switch (current_state) {
    case STATE_INITIALIZING:
      valid_transition = (new_state == STATE_READY || new_state == STATE_ERROR);
      break;
    case STATE_READY:
      valid_transition = (new_state == STATE_MOVING || new_state == STATE_HITTING || new_state == STATE_ERROR);
      break;
    case STATE_MOVING:
      valid_transition = (new_state == STATE_READY || new_state == STATE_ERROR);
      break;
    case STATE_HITTING:
      valid_transition = (new_state == STATE_READY || new_state == STATE_ERROR);
      break;
    case STATE_ERROR:
      valid_transition = (new_state == STATE_READY || new_state == STATE_INITIALIZING);
      break;
  }
  
  if (!valid_transition) {
    String error_details = "Invalid state transition! From: " + String(state_to_string(current_state).c_str()) + 
                          ", To: " + String(state_to_string(new_state).c_str());
    log_error("INVALID_STATE_TRANSITION", error_details.c_str());
    return false;
  }
  
  // 執行狀態轉換
  previous_state = current_state;
  current_state = new_state;
  state_change_time = millis();
  
  // 設定狀態轉換鎖定
  if (new_state == STATE_MOVING || new_state == STATE_HITTING) {
    state_transition_locked = true;
  } else {
    state_transition_locked = false;
  }
  
  Serial.printf("State changed: %s -> %s (%s)\n", 
               state_to_string(previous_state).c_str(), 
               state_to_string(current_state).c_str(), 
               reason);
  
  return true;
}

void check_state_timeout() {
  // 檢查狀態轉換超時
  if (state_transition_locked && 
      (millis() - state_change_time > STATE_CHANGE_TIMEOUT)) {
    String error_details = "State timeout! Current: " + String(state_to_string(current_state).c_str()) + 
                          ", Timeout: " + String(STATE_CHANGE_TIMEOUT) + "ms";
    log_error("STATE_TIMEOUT", error_details.c_str());
    change_state(STATE_ERROR, "State timeout");
  }
}

String state_to_string(SystemState state) {
  switch (state) {
    case STATE_INITIALIZING: return "INITIALIZING";
    case STATE_READY: return "READY";
    case STATE_MOVING: return "MOVING";
    case STATE_HITTING: return "HITTING";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// ===================== 初始化函數 =====================
void setup() {
  Serial.begin(115200);
  set_microros_transports();

  // 設定腳位模式
  pinMode(LED_PIN, OUTPUT);

  pinMode(xstepPin,   OUTPUT);
  pinMode(xdirPin,    OUTPUT);
  pinMode(ystepPin,   OUTPUT);
  pinMode(ydirPin,    OUTPUT);
  pinMode(xenablePin, OUTPUT);
  pinMode(yenablePin, OUTPUT);
  digitalWrite(xenablePin, LOW); // 低電位啟用
  digitalWrite(yenablePin, LOW);

  pinMode(xrightPin, INPUT_PULLDOWN);
  pinMode(ydownPin, INPUT_PULLDOWN);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(limitSwitch, INPUT_PULLUP);  // 敲擊限位
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);

  // ROS 2 初始化
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "move_hit_node", "", &support);

  // 初始化 ROS2 訂閱者
  rclc_subscription_init_default(&subscriber_led, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/ros2_led");
  rclc_subscription_init_default(&subscriber_top_base_locate, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/top_base_locate");
  rclc_subscription_init_default(&subscriber_coordinates, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/World_Coordinates");
  rclc_subscription_init_default(&subscriber_keyboard, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/keyboard_control");
  rclc_subscription_init_default(&subscriber_config, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/system_config");

  // 初始化 ROS2 發布者
  rclc_publisher_init_default(&publisher_locate, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/moveknock_locate_node");
  rclc_publisher_init_default(&publisher_move_hit_initialized, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/move_hit_initialized");

  // 分配訊息緩衝區（使用新的記憶體管理函數）
  if (!allocate_memory()) {
    log_error("MEMORY_ALLOCATION_FAILED", "Critical memory allocation failed during initialization");
    change_state(STATE_ERROR, "Memory allocation failed");
    // 不要 return，繼續初始化其他部分
  }

  // 設定執行器（處理五個訂閱）
  rclc_executor_init(&executor, &support.context, 5, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber_led, &led_msg, &subscription_led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_coordinates, &coordinates_msg, &subscription_coordinates_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_top_base_locate, &top_base_locate_msg, &subscription_top_base_locate_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_keyboard, &keyboard_msg, &subscription_keyboard_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_config, &config_msg, &subscription_config_callback, ON_NEW_DATA);

  // 上線回報 OK
  publish_initialized_ok();

  // 通電後自動 homing
  change_state(STATE_INITIALIZING, "System startup");
  Serial.println("DEBUG: Starting initialization...");
  initializeAxes();
  
  // 檢查 homing 是否成功
  if (current_state == STATE_ERROR) {
    Serial.println("Initialization failed - system in ERROR state");
  } else {
    change_state(STATE_READY, "Initialization completed");
    Serial.printf("Initialized. Current step_delay=%d us\n", (int)step_delay); // 顯示初始化後的移動速度
    
    // 發布初始位置
    publish_position();
  }
  
  Serial.println("DEBUG: Initialization completed, motor enable pins should remain LOW");
  
  // 確保移動馬達保持啟用狀態（不斷電）
  digitalWrite(xenablePin, LOW);
  digitalWrite(yenablePin, LOW);
  Serial.println("FORCE: Movement motors enabled and will remain enabled");
}

// ===================== 主迴圈 =====================
void loop() {
  // 定期執行 ROS2 callback（提高頻率到 10ms）
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
  // 處理指令佇列
  process_command_queue();
  
  // 非阻塞式移動處理
  process_non_blocking_move();
  
  // 記憶體清理
  cleanup_memory();
  
  // 狀態機監控
  check_state_timeout();
  
  // 心跳監控
  if (millis() - last_heartbeat > HEARTBEAT_INTERVAL) {
    last_heartbeat = millis();
    
    // 發布系統狀態
    String status_msg = "";
    switch (current_state) {
      case STATE_INITIALIZING: status_msg = "initializing"; break;
      case STATE_READY: status_msg = "ready"; break;
      case STATE_MOVING: status_msg = "moving"; break;
      case STATE_HITTING: status_msg = "hitting"; break;
      case STATE_ERROR: status_msg = "error"; break;
    }
    
    // 包含指令佇列狀態
    String queue_status = "";
    if (!is_command_queue_empty()) {
      queue_status = ", queue:" + String(queue_count);
    }
    
    // 發布系統狀態和位置
    String full_status = "status:" + status_msg + queue_status + ", pos:X:" + String(current_x, 2) + ",Y:" + String(current_y, 2);
    publish_locate_text(full_status.c_str());
  }
}

// ===================== 敲擊流程（含錄音觸發） =====================
void hitOnce() {
  if (motorRunning) {
    Serial.println("Warning: Hit motor is already running!"); // 警告：敲擊馬達正在運行中！
    return;
  }

  if (current_state == STATE_ERROR) {
    Serial.println("Error: System in error state, cannot execute hit!"); // 錯誤：系統處於錯誤狀態，無法執行敲擊！
    return;
  }

  if (!change_state(STATE_HITTING, "Hit command received")) {
    Serial.println("Failed to change state to HITTING");
    return;
  }
  Serial.println("Starting hit sequence..."); // 開始敲擊流程...

  // 敲擊前先發「開始錄音」，包含磁磚編號和敲擊位置
  String position_names[] = {"left", "mid", "right"};
  String recording_msg = "start_recording:tile_" + String(tile_index) + ":" + position_names[hit_position];
  publish_locate_text(recording_msg.c_str());

  // 啟動敲擊馬達
  startMotor();
  motorRunning = true;
  delay(500);

  unsigned long startTime = millis();

  // 檢查是否已經在限位開關位置
  bool at_limit = (digitalRead(limitSwitch) == LOW);
  bool limit_triggered = false;
  int limit_count = 0;
  
  if (at_limit) {
    Serial.println("Already at limit switch - will complete full rotation");
  } else {
    Serial.println("Starting from current position");
  }

  while (true) {
    // 檢查限位開關
    if (digitalRead(limitSwitch) == LOW) {
      if (!limit_triggered) {
        // 第一次碰到限位開關
        limit_triggered = true;
        limit_count++;
        Serial.printf("Limit switch triggered #%d\n", limit_count);
        
        if (at_limit) {
          // 如果開始時就在限位開關位置，需要完成完整旋轉
          Serial.println("Complete rotation required - continuing...");
        } else {
          // 如果開始時不在限位開關位置，第一次碰到就停止
          stopMotor();
          motorRunning = false;
          change_state(STATE_READY, "Hit completed - motor stopped");
          publish_locate_text("stop_recording");
          Serial.println("Hit completed! Motor stopped, waiting for next hit.");
          break;
        }
      } else {
        // 第二次碰到限位開關（完整旋轉完成）
        limit_count++;
        Serial.printf("Limit switch triggered #%d - complete rotation finished\n", limit_count);
        stopMotor();
        motorRunning = false;
        change_state(STATE_READY, "Hit completed - full rotation finished");
        publish_locate_text("stop_recording");
        Serial.println("Hit completed! Full rotation finished, motor stopped.");
        break;
      }
    }

    // 超時保護
    if (millis() - startTime > hit_timeout) {
      String error_details = "Hit timeout after " + String(millis() - startTime) + "ms, " +
                            "Timeout limit: " + String(hit_timeout) + "ms, " +
                            "Limit count: " + String(limit_count);
      log_error("HIT_TIMEOUT", error_details.c_str());
      stopMotor();
      motorRunning = false;
      change_state(STATE_ERROR, "Hit timeout");
      publish_locate_text("stop_recording");
      Serial.println("Error: Hit timeout!");
      break;
    }
  }
}

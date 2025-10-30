#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>

rcl_subscription_t subscriber_led;
rcl_subscription_t subscriber_coordinates;
std_msgs__msg__Bool led_msg;
std_msgs__msg__String coordinates_msg;
std_msgs__msg__String top_base_locate_msg; 
rcl_subscription_t subscriber_top_base_locate;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher_locate;
std_msgs__msg__String locate_msg; 
rcl_publisher_t publisher_move_hit_initialized;
std_msgs__msg__String move_hit_initialized_msg;
#define LED_PIN 2
bool base_locate_triggered = false; // Flag for /top_base_locate
unsigned long base_locate_timestamp = 0; // Timestamp for /top_base_locate


// 定義腳位
const int xenablePin = 18;
const int yenablePin = 27;
const int xstepPin = 16;
const int xdirPin = 17;
const int ystepPin = 26;
const int ydirPin = 25;
const int xleftPin = 33;  // X軸極限開關
const int ydownPin = 32;  // Y軸極限開關

// 步進馬達參數
const float steps_per_rotation = 200.0 * 16.0; // 1/16步模式的每圈步數
const float hit_of_circle = 14.0;   // 馬達轉一圈棒子移動距離(mm)

// 最大範圍
const float max_x = 300.0; // (mm)
const float max_y = 70.0;  

// 當前座標
float current_x = 0.0;
float current_y = 0.0;

// 敲擊參數
const int encoderPinB = 4;   
const int motorPin1 = 21;    
const int motorPin2 = 22;    
const int limitSwitch = 13;   
bool motorRunning = false;  // 記錄馬達運行狀態
bool triggered = false;     // 記錄微動開關是否觸發



// 設定方向並計算所需步數
struct Movement {
  int stepsRemaining;
  int stepPin;
  int dirPin;
  bool canMove; // 是否允許運動
};

Movement calculateMovement(float current, float target, int stepPin, int dirPin, bool reverseDirection=false) {
  float delta = target - current;
  Movement movement;
  movement.stepsRemaining = abs(delta) * steps_per_rotation / hit_of_circle;
  movement.stepPin = stepPin;
  movement.dirPin = dirPin;
  if (reverseDirection){
    digitalWrite(dirPin, delta > 0 ? LOW : HIGH);
  }else{
    digitalWrite(dirPin, delta > 0 ? HIGH : LOW);
  }
  //digitalWrite(dirPin, delta > 0 ? HIGH : LOW); // 設定方向
  movement.canMove = true; // 初始化為可移動
  return movement;
}

// 移動一步
void moveStep(Movement &movement) {
  if (movement.stepsRemaining > 0 && movement.canMove) {
    digitalWrite(movement.stepPin, HIGH);
    delayMicroseconds(100); // 每個脈衝的延遲時間 (微秒)
    digitalWrite(movement.stepPin, LOW);
    delayMicroseconds(100);
    movement.stepsRemaining--;
  }
}

// 移動到目標位置
void moveToTarget(float target_x, float target_y) {
  Movement xMovement = calculateMovement(current_x, target_x, xstepPin, xdirPin,true);
  Movement yMovement = calculateMovement(current_y, target_y, ystepPin, ydirPin);

  // 確認是否能夠移動X軸和Y軸
  if (digitalRead(xleftPin) == HIGH && digitalRead(xdirPin) == HIGH) { // 向左且碰到左限
    xMovement.canMove = false;
  }
  if (digitalRead(ydownPin) == HIGH && digitalRead(ydirPin) == LOW) { // 向下且碰到下限
    yMovement.canMove = false;
  }

  while (xMovement.stepsRemaining > 0 || yMovement.stepsRemaining > 0) {
    if (xMovement.stepsRemaining > 0) {
      moveStep(xMovement);
    }
    if (yMovement.stepsRemaining > 0) {
      moveStep(yMovement);
    }
  }

  // 更新當前位置
  current_x = target_x;
  current_y = target_y;

  Serial.print("Current Position: (");
  Serial.print(current_x, 2); // 顯示兩位小數
  Serial.print(", ");
  Serial.print(current_y, 2); // 顯示兩位小數
  Serial.println(")");
}

// 同時初始化X軸和Y軸至(0, 0)
void initializeAxes() {
  bool xHomed = false;
  bool yHomed = false;
  
  digitalWrite(xdirPin, HIGH); // 設定X軸向左移動方向
  digitalWrite(ydirPin, LOW); // 設定Y軸向下移動方向

  while (!xHomed || !yHomed) {
    if (!xHomed) {
      digitalWrite(xstepPin, HIGH);
    }
    if (!yHomed) {
      digitalWrite(ystepPin, HIGH);
    }
    delayMicroseconds(100); 
    digitalWrite(xstepPin, LOW);
    digitalWrite(ystepPin, LOW);
    delayMicroseconds(100);

    // 檢查是否達到X軸極限
    if (digitalRead(xleftPin) == HIGH) {
      xHomed = true;
      current_x = 0.0;  // X軸設為0
    }

    // 檢查是否達到Y軸極限
    if (digitalRead(ydownPin) == HIGH) {
      yHomed = true;
      current_y = 0.0;  // Y軸設為0
    }
  }
}

void subscription_led_callback(const void *msg_in) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
  Serial.println("Received bool ");
  if (msg->data) {
    digitalWrite(LED_PIN, HIGH);
    delay(500); 
    digitalWrite(LED_PIN, LOW);
  }
}
void subscription_top_base_locate_callback(const void *msg_in) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
  Serial.print("Received /top_base_locate message: ");
  Serial.println(msg->data.data);

  base_locate_triggered = true; // Set the flag
  base_locate_timestamp = millis(); // Record the timestamp

 
}

void subscription_coordinates_callback(const void *msg_in) {
  if (!base_locate_triggered || (millis() - base_locate_timestamp > 1500)) {
    Serial.println("Ignoring /World_Coordinates as no recent /top_base_locate was received.");
    return;
  }

  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msg_in;
  Serial.println("Received coordinates: ");
  if (msg->data.data) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }

  String input(msg->data.data);
  input.replace("),(", "|");
  input.replace("(", "");
  input.replace(")", "");

  int startIndex = 0;
  int endIndex = input.indexOf('|');

  while (endIndex != -1) {
    String coord = input.substring(startIndex, endIndex);
    processCoordinate(coord);
    startIndex = endIndex + 1;
    endIndex = input.indexOf('|', startIndex);
  }

  // Handle the last coordinate
  if (startIndex < input.length()) {
    String coord = input.substring(startIndex);
    processCoordinate(coord);
  }

  // Move to (0, 0) after completing the task
  moveToTarget(0.0, 0.0);

  // Clear /World_Coordinates memory
  if (coordinates_msg.data.data) {
    free(coordinates_msg.data.data);
    coordinates_msg.data.data = NULL;
    coordinates_msg.data.size = 0;
    coordinates_msg.data.capacity = 100;
  }

  // Clear /top_base_locate memory
  if (top_base_locate_msg.data.data) {
    free(top_base_locate_msg.data.data);
    top_base_locate_msg.data.data = NULL;
    top_base_locate_msg.data.size = 0;
    top_base_locate_msg.data.capacity = 100;

    base_locate_triggered = false; // Reset the flag
  }

  delay(2000);

  // Publish move_hit_initialized
  snprintf(move_hit_initialized_msg.data.data, 50, "move_hit_initialized");
  move_hit_initialized_msg.data.size = strlen(move_hit_initialized_msg.data.data);
  rcl_publish(&publisher_move_hit_initialized, &move_hit_initialized_msg, NULL);
  Serial.println("Published move_hit_initialized to /move_hit_initialized");
  delay(6000);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing micro-ROS...");
  set_microros_transports(); 
  Serial.println("micro-ROS initialized.");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(200);                 
  digitalWrite(LED_PIN, LOW);  

  pinMode(xstepPin, OUTPUT);
  pinMode(xdirPin, OUTPUT);
  pinMode(ystepPin, OUTPUT);
  pinMode(ydirPin, OUTPUT);
  pinMode(xenablePin, OUTPUT);
  digitalWrite(xenablePin, LOW); // 啟用X軸
  pinMode(yenablePin, OUTPUT);
  digitalWrite(yenablePin, LOW); // 啟用Y軸
  pinMode(xleftPin, INPUT_PULLDOWN); // 設置為帶下拉電阻的輸入
  pinMode(ydownPin, INPUT_PULLDOWN); // 設置為帶下拉電阻的輸入
  
  // 敲擊動作相關設定
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(limitSwitch, INPUT_PULLUP);  // 微動開關配置為 INPUT_PULLUP
  // 初始化時馬達不運行
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);

  initializeAxes();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "move_hit_node", "", &support);
  rclc_subscription_init_default(
    &subscriber_led,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/ros2_led");
  rclc_subscription_init_default(
    &subscriber_top_base_locate,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/top_base_locate"
  );
  rclc_subscription_init_default(
    &subscriber_coordinates,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/World_Coordinates"
  );
  rclc_publisher_init_default(
    &publisher_locate,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/moveknock_locate_node"
  );
  rclc_publisher_init_default(
    &publisher_move_hit_initialized,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/move_hit_initialized"
  );
  coordinates_msg.data.data = (char *)malloc(100 * sizeof(char));  // Allocate memory for 100 characters
  coordinates_msg.data.size = 0;  // Set initial size
  coordinates_msg.data.capacity = 100;  // Set capacity
  move_hit_initialized_msg.data.data = (char *)malloc(50 * sizeof(char)); // Allocate memory
  move_hit_initialized_msg.data.size = 0;  // Set initial size
  move_hit_initialized_msg.data.capacity = 50;  // Set capacity
  top_base_locate_msg.data.data = (char *)malloc(100 * sizeof(char)); // Allocate memory
  top_base_locate_msg.data.size = 0;  // Set initial size
  top_base_locate_msg.data.capacity = 100;  // Set capacity
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber_led, &led_msg, &subscription_led_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_coordinates, &coordinates_msg, &subscription_coordinates_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber_top_base_locate, &top_base_locate_msg, &subscription_top_base_locate_callback, ON_NEW_DATA);
  Serial.println("OK");
  // 同時初始化X軸和Y軸至 (0,0)
    Serial.println("OK2");
}
// 敲擊函數，每次呼叫執行一次敲擊動作
void hitOnce() {
  if (!motorRunning) {
    Serial.println("Starting motor for hitting action.");
    startMotor();  // 啟動馬達

    // 先運行 2 秒
    delay(500);

    // 2 秒後開始檢查微動開關狀態，直到觸發後停止馬達並退出
    while (true) {
      if (digitalRead(limitSwitch) == LOW) {  // 當微動開關被觸發時
        stopMotor();  // 停止馬達
        Serial.println("Hit action completed. Limit switch triggered.");
        break;  // 跳出 while 循環，結束函數
      }
    }
  } else {
    Serial.println("Motor is already running.");
  }
}

// 启动马达逆时针转动
void startMotor() {
  digitalWrite(motorPin1, HIGH);  // 马达逆时针转动
  digitalWrite(motorPin2, LOW);
  motorRunning = true;
  triggered = false;  // 重置微動開關觸發標誌
}

// 停止马达
void stopMotor() {
  digitalWrite(motorPin1, LOW);  // 停止马达
  digitalWrite(motorPin2, LOW);
  motorRunning = false;
}

void loop() {
   if (base_locate_triggered && (millis() - base_locate_timestamp > 2000)) {
    base_locate_triggered = false;
    Serial.println("/top_base_locate flag reset after timeout.");
   }
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // 切分輸入座標
    input.replace("),(", "|"); // 用 | 分隔座標
    input.replace("(", "");    // 移除左括號
    input.replace(")", "");    // 移除右括號
    
    int startIndex = 0;
    int endIndex = input.indexOf('|');

    while (endIndex != -1) {
      String coord = input.substring(startIndex, endIndex);
      processCoordinate(coord);
      startIndex = endIndex + 1;
      endIndex = input.indexOf('|', startIndex);
    }

    // 處理最後一個座標
    if (startIndex < input.length()) {
      String coord = input.substring(startIndex);
      processCoordinate(coord);
      //free(coordinates_msg.data.data);
      //free(top_base_locate_msg.data.data);
    }

    // 返回原點
    moveToTarget(0.0, 0.0);
       // 釋放 coordinates_msg 的記憶體
       
  }
  
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(100);

  if (Serial.available()>0){
    int command =Serial.read();
    if(command=='h'){
      hitOnce();
    }
  }
}

// 處理單一座標字串並移動到該座標
void processCoordinate(String coord) {
  int separatorIndex = coord.indexOf(',');

  if (separatorIndex != -1) {
    float x = coord.substring(0, separatorIndex).toFloat();
    float y = coord.substring(separatorIndex + 1).toFloat();

    if (x >= 0.0 && x <= max_x && y >= 0.0 && y <= max_y) {
      moveToTarget(x, y);

      // Create and publish message to /moveknock_locate_node
      locate_msg.data.data = (char *)malloc(50 * sizeof(char));
      snprintf(locate_msg.data.data, 50, "Target located at X: %.2f, Y: %.2f", x, y);
      locate_msg.data.size = strlen(locate_msg.data.data);
      locate_msg.data.capacity = 100;
      rcl_publish(&publisher_locate, &locate_msg, NULL); // Publish to the topic
      delay(1000);
      Serial.println("Published location to /moveknock_locate_node");

      hitOnce();   // 執行敲擊動作
      delay(5000);       // 延遲10秒
      

    } else {
      Serial.println("Error: Coordinates out of range.");
    }
  } else {
    Serial.println("Error: Invalid input format.");
  }
}


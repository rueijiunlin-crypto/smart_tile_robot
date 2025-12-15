#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "math.h"

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

// 發送單一脈衝
void sendSinglePulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(100);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(100);
}

// ==========================================
// 馬達控制核心
// ==========================================
void setTargetPositions(float new_L1, float new_L2) {
  // 1. 計算繩長變化量
  float delta_L1 = new_L1 - old_L1;
  float delta_L2 = new_L2 - old_L2;

  // 2. 計算步數
  int steps_x = abs(delta_L1 * x_correction_factor * steps_per_rotation / ropeLength_of_circle) * 10;
  int steps_y = abs(delta_L2 * steps_per_rotation / ropeLength_of_circle) * 10;

  // 3. 計算延遲 (線性插補)
  unsigned long pulseDelay_x = (steps_x > 0) ? (500000 / steps_x) : 0;
  unsigned long pulseDelay_y = (steps_y > 0) ? (500000 / steps_y) : 0;

  // 4. 設置方向
  // 左邊(L1): >=0 為正轉(放繩)
  // 右邊(L2): <=0 為正轉(放繩) [這是依照您的特殊硬體邏輯]
  bool direction_x = delta_L1 >= 0; 
  bool direction_y = delta_L2 <= 0; 

  digitalWrite(xdirPin, direction_x);
  digitalWrite(ydirPin, direction_y);

  // 5. 同步驅動兩個馬達
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

    // 6. 輸出位置資訊
    Serial.print("Moved to: (");
    Serial.print(current_x_disp);
    Serial.print(", ");
    Serial.print(current_y_disp);
    Serial.println(")");

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
  delay(1000);  // 等待 Serial 穩定
  
  Serial.println("\n\n=== 上機構啟動（序列埠控制模式）===");
  Serial.println("指令說明：");
  Serial.println("  w 或 t : 向上移動 (Y 軸 -1)");
  Serial.println("  s 或 g : 向下移動 (Y 軸 +1)");
  Serial.println("  a      : 向左移動 (X 軸 -1)");
  Serial.println("  d      : 向右移動 (X 軸 +1)");
  Serial.println("請在序列埠監視器中輸入指令...\n");

  // === [新算法初始化] ===
  // 假設開機時機器人在正中間，高度為 h (30cm)
  float start_term_x = W / 2.0;
  float start_term_y = h;
  float initial_length = sqrt(pow(start_term_x, 2) + pow(start_term_y, 2));
  
  old_L1 = initial_length;
  old_L2 = initial_length;
  current_x_disp = 0;
  current_y_disp = 0;

  Serial.print("初始位置: (");
  Serial.print(current_x_disp);
  Serial.print(", ");
  Serial.print(current_y_disp);
  Serial.println(")");
  Serial.println("準備接收指令...\n");
}

// ==========================================
// Loop
// ==========================================

void loop() {
  // 1. 從序列埠讀取指令
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // 移除前後空白
    
    if (input.length() > 0) {
      Serial.print("收到指令: ");
      Serial.println(input);
      
      // 將輸入的字串中的每個字元加入佇列
      for (int i = 0; i < input.length(); i++) {
        char command = input.charAt(i);
        if (command == 'w' || command == 't' || command == 's' || command == 'g' || 
            command == 'a' || command == 'd') {
          if (!enqueueCommand(command)) {
            Serial.println("警告: 命令佇列已滿！");
          }
        } else if (command != ' ' && command != '\n' && command != '\r') {
          Serial.print("無效指令: '");
          Serial.print(command);
          Serial.println("' (忽略)");
        }
      }
    }
  }

  // 2. 處理佇列命令 (Queue Processing)
  static bool isProcessing = false;

  if (!isProcessing && !isQueueEmpty()) {
    char command;
    if (dequeueCommand(&command)) {
      isProcessing = true;
      Serial.print("執行指令: ");
      Serial.println(command);

      // === 呼叫統一的移動邏輯 ===
      executeMoveLogic(command);
      // =======================

      lastMoveTime = millis();

      // 移動間隔等待
      unsigned long processStartTime = millis();
      while (millis() - processStartTime < moveInterval) {
        // 在等待期間也可以處理序列埠輸入
        if (Serial.available() > 0) {
          String input = Serial.readStringUntil('\n');
          input.trim();
          if (input.length() > 0) {
            for (int i = 0; i < input.length(); i++) {
              char cmd = input.charAt(i);
              if (cmd == 'w' || cmd == 't' || cmd == 's' || cmd == 'g' || 
                  cmd == 'a' || cmd == 'd') {
                enqueueCommand(cmd);
              }
            }
          }
        }
      }
      isProcessing = false;
    }
  }

  // 3. 定期輸出當前座標 (每 5 秒)
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint >= 5000) {
    Serial.print("當前座標: (");
    Serial.print(current_x_disp);
    Serial.print(", ");
    Serial.print(current_y_disp);
    Serial.println(")");
    lastStatusPrint = millis();
  }
}

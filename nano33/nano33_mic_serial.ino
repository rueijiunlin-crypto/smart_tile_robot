// Nano 33 BLE Sense Rev2 - PDM 麥克風經 USB 串列傳輸 16-bit PCM
// 指令："START" / "STOP"（以 \n 結尾）
// 封包：'A''U''D''0' + uint16 小端長度 + PCM 資料

#include <PDM.h>

static const int SAMPLE_RATE = 22050;     // 可改為 16000
static const int CHUNK_SAMPLES = 512;     // 每幀 512 samples → 1024 bytes
static const uint32_t BAUD = 2000000;     // USB CDC 虛擬串列，2 Mbps

volatile bool recording = false;
int16_t pcmBuffer[CHUNK_SAMPLES];

void setup() {
  Serial.begin(BAUD);
  while (!Serial) { ; }

  PDM.onReceive(onPDMData);
  PDM.setGain(30);
  // 延後 PDM.begin 到收到 START
}

void loop() {
  // 處理主控端命令
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "START" && !recording) {
      if (PDM.begin(1, SAMPLE_RATE)) {
        recording = true;
        Serial.write("OK_START\n");
      } else {
        Serial.write("ERR_PDM\n");
      }
    } else if (cmd == "STOP" && recording) {
      recording = false;
      PDM.end();
      Serial.write("OK_STOP\n");
    }
  }

  // 將 PDM 可用資料轉為封包輸出
  if (recording) {
    int available = PDM.available();
    // 每次讀取 CHUNK_SAMPLES*2 bytes
    while (available >= CHUNK_SAMPLES * (int)sizeof(int16_t)) {
      int readBytes = PDM.read(pcmBuffer, CHUNK_SAMPLES * sizeof(int16_t));
      if (readBytes == CHUNK_SAMPLES * (int)sizeof(int16_t)) {
        uint8_t header[6] = {'A','U','D','0', (uint8_t)(readBytes & 0xFF), (uint8_t)((readBytes >> 8) & 0xFF)};
        Serial.write(header, 6);
        Serial.write((uint8_t*)pcmBuffer, readBytes);
      }
      available -= readBytes;
    }
  }
}

void onPDMData() {
  // 使用輪詢讀取，不在中斷內做大量工作
}



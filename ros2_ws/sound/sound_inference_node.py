#!/usr/bin/env python3
import os
import time
import csv
import numpy as np
import soundfile as sf
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tflite_runtime.interpreter import Interpreter


# === 參數設定（專案內相對路徑，基於此檔案所在目錄）===
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(_SCRIPT_DIR, 'tflite-model', 'model.tflite')
RECORDINGS_ROOT = os.path.join(_SCRIPT_DIR, 'recordings')
LABELS = ["normal", "broken"]  # 依照 Edge Impulse 模型


class SoundInferenceNode(Node):
    def __init__(self):
        super().__init__('sound_inference_node')
        self.publisher_ = self.create_publisher(String, '/tile_sound_result', 10)
        self.get_logger().info("✅ Sound Inference Node 啟動")

        # 鎖定此次啟動要掃描的錄音資料夾（選 recordings/ 下最新建立的資料夾）
        self.record_dir = self._pick_latest_recording_dir()
        if self.record_dir is None:
            # 若沒有子資料夾，則使用根目錄（仍可處理直接放在 recordings/ 的檔案）
            self.record_dir = RECORDINGS_ROOT
            self.get_logger().warn(f"未找到時間戳錄音資料夾，退回掃描根目錄：{self.record_dir}")
        else:
            self.get_logger().info(f" 本次僅掃描此資料夾：{self.record_dir}")

        # 結果 CSV 儲存在同一資料夾
        self.result_csv = os.path.join(self.record_dir, 'inference_results.csv')

        # 初始化模型
        self.interpreter = Interpreter(model_path=MODEL_PATH)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # 模型 I/O 型態與量化參數（自動偵測）
        self.input_dtype = self.input_details[0]['dtype']
        self.output_dtype = self.output_details[0]['dtype']

        def _qparam(d):
            qp = d.get('quantization_parameters', {})
            scales = qp.get('scales', np.array([], dtype=np.float32))
            zeros = qp.get('zero_points', np.array([], dtype=np.int32))
            s = float(scales[0]) if scales.size > 0 else 0.0
            z = int(zeros[0]) if zeros.size > 0 else 0
            return s, z

        self.input_scale, self.input_zero_point = _qparam(self.input_details[0])
        self.output_scale, self.output_zero_point = _qparam(self.output_details[0])

        # 取得輸入長度（多數為 650；依模型 shape 為準）
        self.input_length = int(self.input_details[0]['shape'][1])

        self.get_logger().info(
            f"模型輸入長度：{self.input_length}，input_dtype={self.input_dtype.__name__}, "
            f"output_dtype={self.output_dtype.__name__}, input_q=({self.input_scale},{self.input_zero_point}), "
            f"output_q=({self.output_scale},{self.output_zero_point})"
        )

        # 已處理過的檔案
        self.processed = set()

        # 每 2 秒掃描一次資料夾
        self.timer = self.create_timer(2.0, self.scan_folder)

    # === 掃描錄音資料夾 ===
    def scan_folder(self):
        for root, _, files in os.walk(self.record_dir):
            for f in files:
                if f.endswith(".wav"):
                    full_path = os.path.join(root, f)
                    if full_path not in self.processed:
                        self.processed.add(full_path)
                        self.get_logger().info(f"🔍 偵測到新錄音：{f}")
                        try:
                            label, conf = self.classify_wav(full_path)
                            self.save_result(f, label, conf)
                            self.publish_result(f, label, conf)
                            self.get_logger().info(f"✅ {f} → {label} ({conf*100:.1f}%)")
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ {f} 推論失敗：{e}")

    # === 執行模型推論 ===
    def classify_wav(self, wav_path):
        data, fs = sf.read(wav_path)
        data = np.asarray(data, dtype=np.float32)
        if data.ndim > 1:
            data = data[:, 0]
        if fs != 44100:
            self.get_logger().warn(
                f"⚠️ {os.path.basename(wav_path)} 取樣率為 {fs}Hz，預期為 44100Hz；將直接使用原始資料"
            )
        
        # 將音訊調整至模型期望的長度（例如 650）
        # 方法：線性插值或補零
        if len(data) > self.input_length:
            indices = np.linspace(0, len(data) - 1, self.input_length)
            data = np.interp(indices, np.arange(len(data)), data)
        elif len(data) < self.input_length:
            # 如果太短，補零
            padded = np.zeros(self.input_length, dtype=np.float32)
            padded[:len(data)] = data
            data = padded
        
        if self.input_dtype == np.int8:
            # 量化：將 float32 轉換為 int8
            if self.input_scale == 0:
                raise ValueError("模型輸入為 int8 但缺少量化參數（scale=0）")
            quantized_input = np.round(data / self.input_scale + self.input_zero_point).astype(np.int8)
            quantized_input = np.clip(quantized_input, -128, 127)
            tensor_in = quantized_input.reshape(1, -1)
        else:
            # 直接使用 float32
            tensor_in = data.reshape(1, -1).astype(np.float32)

        # 設定輸入（reshape 為 [1, input_length]）
        self.interpreter.set_tensor(self.input_details[0]['index'], tensor_in)
        self.interpreter.invoke()

        # 取得輸出
        raw_out = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        if self.output_dtype == np.int8:
            # 反量化：將 int8 轉回 float32
            if self.output_scale == 0:
                # 退而求其次，直接轉 float，後續 softmax
                output = raw_out.astype(np.float32)
            else:
                output = (raw_out.astype(np.float32) - self.output_zero_point) * self.output_scale
        else:
            output = raw_out.astype(np.float32)

        # 應用 softmax（將 logits 轉為機率）
        exp_output = np.exp(output - np.max(output))
        denom = np.sum(exp_output)
        output = exp_output / denom if denom > 0 else exp_output

        label = LABELS[int(np.argmax(output))]
        confidence = float(np.max(output))
        return label, confidence

    # === 寫入 CSV ===
    def save_result(self, filename, label, confidence):
        file_exists = os.path.isfile(self.result_csv)
        with open(self.result_csv, mode='a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(["timestamp", "filename", "label", "confidence"])
            writer.writerow([
                time.strftime("%Y-%m-%d %H:%M:%S"),
                filename,
                label,
                f"{confidence:.4f}"
            ])

    # === 選取最新錄音資料夾 ===
    def _pick_latest_recording_dir(self):
        try:
            if not os.path.isdir(RECORDINGS_ROOT):
                return None
            candidates = []
            for name in os.listdir(RECORDINGS_ROOT):
                p = os.path.join(RECORDINGS_ROOT, name)
                if os.path.isdir(p):
                    try:
                        mtime = os.path.getmtime(p)
                    except Exception:
                        mtime = 0.0
                    candidates.append((mtime, p))
            if not candidates:
                return None
            candidates.sort(reverse=True)
            return candidates[0][1]
        except Exception:
            return None

    # === 發布 ROS 話題 ===
    def publish_result(self, filename, label, confidence):
        msg = String()
        msg.data = f"{filename}: {label} ({confidence*100:.1f}%)"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SoundInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 結束 Sound Inference Node")
    except Exception as e:
        node.get_logger().error(f"節點執行錯誤: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

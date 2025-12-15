#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoHit Vision Node
用途：
 - 使用 RealSense 相機擷取畫面
 - 進行影像辨識找出磁磚區域
 - 自動計算三個拍打位置
 - 發布對應的 ROS2 topic (World_Coordinates / keyboard_control / vision_status)
"""

import cv2
import numpy as np
import time
import json
import os
import pyrealsense2 as rs   # ✅ RealSense SDK

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AutoHitVisionNode(Node):
    """ROS2 節點：負責影像偵測與打擊點座標發送"""

    def __init__(self):
        super().__init__('auto_hit_vision_node')

        # ======================== ROS2 Topic ========================
        # 發布目標世界座標
        #self.coordinates_pub = self.create_publisher(String, '/World_Coordinates', 10)
        # 發布鍵盤模擬控制（如 t/h/n/e）
        #self.keyboard_pub    = self.create_publisher(String, '/keyboard_control', 10)
        # 發布影像狀態（是否找到目標）
        #self.status_pub      = self.create_publisher(String, '/vision_status', 10)
        # 訂閱外部配置參數
        self.create_subscription(String, '/system_config', self.config_callback, 10)
        # 訂閱運動控制回報狀態（ready / moving）
        self.create_subscription(String, '/moveknock_locate_node', self.locate_status_callback, 10)

        # ======================== 偵測參數 ========================
        self.image_scale = 1.0          # 不縮放，使用原解析度
        self.min_area = 6000
        self.max_area = 250000
        self.check_aspect_ratio = False
        self.min_aspect_ratio = 3.0
        self.max_aspect_ratio = 12.0

        # 柔性矩形模式（允許非完全矩形）
        self.soft_rect_mode = True
        self.soft_rect_min_area = 6000
        self.soft_rect_max_area = 300000
        self.soft_rect_min_vertices = 4
        self.soft_rect_max_vertices = 8
        self.soft_rect_min_rectangularity = 0.60
        self.soft_rect_min_solidity      = 0.80
        self.soft_rect_parallel_tol_deg  = 15.0

        # ======================== 座標對應 ========================
        # RealSense 採樣畫面大小
        self.image_width = 640
        self.image_height = 480
        # 對應 ESP32 工作範圍（mm 或 cm）
        self.workspace_x_min = 155.0
        self.workspace_x_max = self.workspace_x_min + 260.0  # 155 → 415 mm
        self.workspace_y_min = 295.0
        self.workspace_y_max = self.workspace_y_min + 96.0   # 295 → 391 mm

        # ======================== 單應性（平面校正） ========================
        self.homography_H = None                 # 3x3 單應性矩陣（像素 → 世界mm）
        self.h_file = os.path.join(os.path.dirname(__file__), 'homography.npy')
        self.calib_mode = False                  # 是否進入滑鼠點選四點校正模式
        self.calib_img_points = []               # 暫存使用者點選之影像點
        # 預設世界座標四點（依序：左下、右下、右上、左上）
        self.calib_world_points = np.array([
            [self.workspace_x_min, self.workspace_y_min],
            [self.workspace_x_max, self.workspace_y_min],
            [self.workspace_x_max, self.workspace_y_max],
            [self.workspace_x_min, self.workspace_y_max],
        ], dtype=np.float32)
        self.calib_background = None             # 校正時顯示的凍結畫面

        # ======================== 鎖定與流程控制 ========================
        self.start_center_thresh_px = 60.0   # 啟動條件：距中心距離閾值
        self.lock_reacquire_thresh_px = 80.0
        self.hit_sequence_active = False     # 是否在執行打擊序列
        self.hit_index = 0                   # 打擊第幾點（1~3）
        self.target_points_img = []          # 儲存三個目標影像座標
        self.last_status = 'ready'           # 來自運動節點的狀態
        self.tile_index = 1                  # 當前磁磚編號
        self.lock_rect = None
        self.lock_last_seen = 0.0
        self.lock_timeout = 2.0
        self.stable_detection_time = 1.2
        self.first_detection_time = 0.0

        # FSM（有限狀態機）控制拍打流程
        self.seq_phase = 'IDLE'              # IDLE, MOVING, WAIT_HIT
        self.cmd_time  = 0.0
        self.min_move_settle = 0.25          # 等待移動穩定時間
        self.min_hit_gap     = 0.20          # 打擊間隔時間

        # ======================== RealSense 初始化 ========================
        self.rs_pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.rs_pipe.start(cfg)

        # 預先建立視窗，確保可綁定滑鼠回呼
        try:
            cv2.namedWindow('AutoHit Vision - Color')
            cv2.namedWindow('AutoHit Vision - Processed (Gray+Edges)')
        except Exception:
            pass

        # 關閉紅外線投影器避免干擾
        for s in profile.get_device().sensors:
            if s.supports(rs.option.emitter_enabled):
                s.set_option(rs.option.emitter_enabled, 0)
            if s.supports(rs.option.laser_power):
                s.set_option(rs.option.laser_power, 0)
        self.get_logger().info('✅ RealSense color stream started (IR emitter off)')

        # 每 10 ms 執行一次主循環
        self.create_timer(0.01, self.loop_once)

        # 嘗試載入既有的單應性
        self.load_homography()
        if self.homography_H is None:
            self.get_logger().info('提示：按下 c 進行四點校正（BL→BR→TR→TL）以建立 homography.npy')

    # ======================== 取得一張影像 ========================
    def get_bgr_frame(self):
        """從 RealSense 擷取彩色影像 (BGR8)"""
        frames = self.rs_pipe.wait_for_frames()
        color = frames.get_color_frame()
        if not color:
            return None
        return np.asanyarray(color.get_data())

    # ======================== ROS Callback ========================
    def publish_status(self, status: str):
        """發布目前影像狀態 (e.g. no_target)"""
       #  self.status_pub.publish(String(data=status))  # ✅ 註解掉，避免重複發布
        pass

    def config_callback(self, msg: String):
        """接收外部設定 JSON 資料以動態調整參數"""
        try:
            data = json.loads(msg.data)
            if data.get('type') == 'vision':
                if 'soft_rect_mode' in data:
                    self.soft_rect_mode = bool(data['soft_rect_mode'])
                if 'soft_rect_min_rectangularity' in data:
                    self.soft_rect_min_rectangularity = float(data['soft_rect_min_rectangularity'])
                if 'soft_rect_min_solidity' in data:
                    self.soft_rect_min_solidity = float(data['soft_rect_min_solidity'])
                if 'soft_rect_parallel_tol_deg' in data:
                    self.soft_rect_parallel_tol_deg = float(data['soft_rect_parallel_tol_deg'])
                if 'min_area' in data:
                    self.min_area = int(data['min_area'])
                if 'max_area' in data:
                    self.max_area = int(data['max_area'])
        except Exception as e:
            self.get_logger().warn(f'Config parse error: {e}')

    def locate_status_callback(self, msg: String):
        """接收運動節點的狀態 (如 'status:ready')"""
        data = msg.data.strip()
        if data.startswith('status:'):
            self.last_status = data.split(':', 1)[1]

    # ======================== 幾何與輔助函式 ========================
    def image_to_workspace(self, ix: float, iy: float):
        """影像座標 → 工作座標
        1) 若有單應性 H，使用透視變換
        2) 否則退回線性映射
        """
        if self.homography_H is not None:
            pts = np.array([[[float(ix), float(iy)]]], dtype=np.float32)
            mapped = cv2.perspectiveTransform(pts, self.homography_H)
            wx, wy = float(mapped[0, 0, 0]), float(mapped[0, 0, 1])
            return wx, wy
        # 線性對應（後備方案）
        wx = (ix / self.image_width)  * (self.workspace_x_max - self.workspace_x_min) + self.workspace_x_min
        wy = (iy / self.image_height) * (self.workspace_y_max - self.workspace_y_min) + self.workspace_y_min
        return wx, wy

    # ======================== 單應性：載入/儲存/計算 ========================
    def load_homography(self):
        try:
            if os.path.exists(self.h_file):
                H = np.load(self.h_file)
                if isinstance(H, np.ndarray) and H.shape == (3, 3):
                    self.homography_H = H.astype(np.float32)
                    if not self._validate_homography_matrix(self.homography_H):
                        self.get_logger().warn('Loaded H failed validation; please re-calibrate (press c)')
                    else:
                        self.get_logger().info('Loaded homography H from file')
                else:
                    self.get_logger().warn('Invalid homography file format; ignoring')
            else:
                self.get_logger().info('No homography file found; using linear mapping')
        except Exception as e:
            self.get_logger().warn(f'Load homography failed: {e}')

    def save_homography(self):
        if self.homography_H is None:
            return
        try:
            np.save(self.h_file, self.homography_H)
            self.get_logger().info(f'Saved homography to {self.h_file}')
        except Exception as e:
            self.get_logger().warn(f'Save homography failed: {e}')

    def _compute_h_from_clicks(self):
        if len(self.calib_img_points) < 4:
            return False
        img_pts = np.array(self.calib_img_points[:4], dtype=np.float32)
        wld_pts = self.calib_world_points
        H, mask = cv2.findHomography(img_pts, wld_pts, method=cv2.RANSAC, ransacReprojThreshold=2.0)
        if H is None:
            self.get_logger().warn('findHomography failed')
            return False
        self.homography_H = H.astype(np.float32)
        # 校驗 H（簡單重投影誤差）
        if not self._validate_homography_points(img_pts, wld_pts, self.homography_H):
            self.get_logger().warn('Calibration residual too large; please re-calibrate (ensure correct BL→BR→TR→TL order)')
        self.save_homography()
        self.get_logger().info('Homography computed and saved. Calibration finished.')
        return True

    def _validate_homography_matrix(self, H: np.ndarray) -> bool:
        try:
            # 行列式不可太小（避免奇異）
            det = np.linalg.det(H[:2, :2])
            return np.isfinite(det) and abs(det) > 1e-6
        except Exception:
            return False

    def _validate_homography_points(self, img_pts: np.ndarray, wld_pts: np.ndarray, H: np.ndarray) -> bool:
        try:
            pred = cv2.perspectiveTransform(img_pts.reshape(1, -1, 2), H).reshape(-1, 2)
            err = np.linalg.norm(pred - wld_pts, axis=1)
            mean_err = float(np.mean(err))
            self.get_logger().info(f'Calibration reprojection mean error: {mean_err:.2f} mm')
            return mean_err < 5.0
        except Exception:
            return False

    # ======================== 單應性：互動式校正（滑鼠四點） ========================
    def start_calibration(self, color_frame_vis):
        self.calib_mode = True
        self.calib_img_points = []
        self.calib_background = color_frame_vis.copy()
        # 綁定滑鼠回呼（若視窗尚未建立，已於 __init__ 嘗試建立）
        cv2.setMouseCallback('AutoHit Vision - Color', self._on_mouse_calib)
        # 暫停序列流程，避免畫面動態干擾
        self.hit_sequence_active = False
        self.seq_phase = 'IDLE'
        self.get_logger().info('Window frozen. Start calibration: click BL, BR, TR, TL in order')

    def _on_mouse_calib(self, event, x, y, flags, param=None):
        if not self.calib_mode:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.calib_img_points) < 4:
                self.calib_img_points.append([float(x), float(y)])
                self.get_logger().info(f'Clicked point {len(self.calib_img_points)} at ({x}, {y})')
            if len(self.calib_img_points) == 4:
                ok = self._compute_h_from_clicks()
                self.calib_mode = False
                # 清除回調避免誤點
                try:
                    cv2.setMouseCallback('AutoHit Vision - Color', lambda *args: None)
                except Exception:
                    pass

    @staticmethod
    def rect_long_axis(rect):
        """計算最長軸方向向量"""
        (cx, cy), (w, h), angle = rect
        if w < h:
            w, h = h, w
            angle += 90.0
        rad = np.deg2rad(angle)
        ux, uy = np.cos(rad), np.sin(rad)
        return (cx, cy), (w, h), (ux, uy)

    @staticmethod
    def third_centers_along_long_axis(rect):
        """沿著長軸方向取三個等距中心點 (左/中/右)"""
        (cx, cy), (w, h), (ux, uy) = AutoHitVisionNode.rect_long_axis(rect)
        return [(cx + ux*d, cy + uy*d) for d in [-w/3.0, 0.0, w/3.0]]

    # ======================== 偵測矩形目標 ========================
    def find_candidate_rects(self, proc_img, draw_img):
        """尋找所有矩形候選區域"""
        contours, _ = cv2.findContours(proc_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 根據模式檢查面積範圍
            if self.soft_rect_mode:
                if not (self.soft_rect_min_area < area < self.soft_rect_max_area):
                    continue
            else:
                if not (self.min_area < area < self.max_area):
                    continue

            # 計算凸包與緊實度
            hull = cv2.convexHull(cnt)
            hull_area = max(1.0, cv2.contourArea(hull))
            solidity = area / hull_area

            # 多邊形近似
            eps = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, eps, True)

            if self.soft_rect_mode:
                # 頂點數限制
                if not (self.soft_rect_min_vertices <= len(approx) <= self.soft_rect_max_vertices):
                    continue
                rect = cv2.minAreaRect(approx)
                (w, h) = rect[1]
                if w <= 0 or h <= 0:
                    continue
                rectangularity = area / (w * h)
                if rectangularity < self.soft_rect_min_rectangularity:
                    continue
                if solidity < self.soft_rect_min_solidity:
                    continue
            else:
                # 僅接受四邊形且凸多邊形
                if len(approx) != 4 or not cv2.isContourConvex(approx):
                    continue
                rect = cv2.minAreaRect(approx)
                (w, h) = rect[1]
                if w <= 0 or h <= 0:
                    continue

            # 繪製可視化方框
            box = cv2.boxPoints(rect).astype(np.int32)
            cv2.drawContours(draw_img, [box], 0, (0, 255, 255), 2)
            candidates.append(rect)

        return candidates

    def choose_center_most(self, rects, img_w, img_h):
        """選擇距畫面中心最近的矩形"""
        if not rects:
            return None
        cx0, cy0 = img_w/2.0, img_h/2.0
        return min(rects, key=lambda r: (r[0][0]-cx0)**2 + (r[0][1]-cy0)**2)

    def detect_and_lock_target(self, frame):
        """執行影像偵測、找主矩形並可視化"""
        color = cv2.resize(frame, (0, 0), fx=self.image_scale, fy=self.image_scale)
        gray  = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        # 邊緣檢測流程
        blur = cv2.GaussianBlur(gray, (7,7), 0)
        edges = cv2.Canny(blur, 50, 120)
        proc  = cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                                 cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)), iterations=1)
        proc_bgr = cv2.cvtColor(proc, cv2.COLOR_GRAY2BGR)

        img_h, img_w = color.shape[:2]
        cx_img, cy_img = img_w/2.0, img_h/2.0

        # 找出候選矩形
        candidates = self.find_candidate_rects(proc, proc_bgr)
        main_rect = self.choose_center_most(candidates, img_w, img_h)

        # 可視化主矩形
        if main_rect is not None:
            box = cv2.boxPoints(main_rect).astype(np.int32)
            for vis in (proc_bgr, color):
                cv2.drawContours(vis, [box], 0, (0, 255, 0), 3)
                (rcx, rcy) = main_rect[0]
                cv2.circle(vis, (int(rcx), int(rcy)), 5, (0, 255, 0), -1)

        # 標示畫面中心
        for vis in (proc_bgr, color):
            cv2.circle(vis, (int(cx_img), int(cy_img)), 5, (0, 0, 255), -1)

        return proc_bgr, color, candidates, main_rect, (cx_img, cy_img)

    # ======================== 主循環 ========================
    def loop_once(self):
        """每次循環處理一幀影像與自動拍擊流程"""
        frame = self.get_bgr_frame()
        if frame is None:
            self.get_logger().error('Camera read failed')
            return

        proc_vis, color_vis, rects, main_rect, (cx_img, cy_img) = self.detect_and_lock_target(frame)

        # 校正模式：只顯示凍結畫面，不執行偵測/發佈/狀態機
        if self.calib_mode and self.calib_background is not None:
            # 確保回呼持續綁定
            try:
                cv2.setMouseCallback('AutoHit Vision - Color', self._on_mouse_calib)
            except Exception:
                pass
            vis = self.calib_background.copy()
            guide = [
                'Calibration: click 4 points in order',
                '1) Bottom-Left  2) Bottom-Right  3) Top-Right  4) Top-Left',
                f'Clicked: {len(self.calib_img_points)}/4',
                'Keys: r=reset H, l=load H, q=quit'
            ]
            y0 = 24
            for i, line in enumerate(guide):
                cv2.putText(vis, line, (10, y0 + i*22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            for i, (px, py) in enumerate(self.calib_img_points):
                cv2.circle(vis, (int(px), int(py)), 6, (0, 255, 255), -1)
            cv2.imshow('AutoHit Vision - Color', vis)
            # 不更新處理視窗，達到凍結效果
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rclpy.shutdown()
            elif key == ord('r'):
                self.homography_H = None
                self.get_logger().info('Homography reset; fallback to linear mapping')
            elif key == ord('l'):
                self.load_homography()
            return

        # --- 若偵測到矩形 ---
        if main_rect is not None:
            # 取得三個打擊點（影像座標）
            self.target_points_img = self.third_centers_along_long_axis(main_rect)
            for p in self.target_points_img:
                for vis in (proc_vis, color_vis):
                    cv2.circle(vis, (int(p[0]), int(p[1])), 5, (255, 0, 0), -1)

            (rcx, rcy) = main_rect[0]
            dist_center = np.hypot(rcx - cx_img, rcy - cy_img)
            # 若主矩形接近畫面中心 → 啟動打擊序列
            if (not self.hit_sequence_active) and (dist_center < self.start_center_thresh_px):
                #self.keyboard_pub.publish(String(data='t'))  # 啟動信號
                self.hit_sequence_active = True
                self.hit_index = 0
                self.seq_phase = 'IDLE'
                self.get_logger().info(f'Hit sequence started for tile {self.tile_index}')
        else:
            # 若沒找到 → 顯示提示
            #self.publish_status('no_target')
            for vis in (proc_vis, color_vis):
                cv2.putText(vis, 'No target detected', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # ======================== 打擊 FSM 流程 ========================
        if self.hit_sequence_active and len(self.target_points_img) == 3:
            # (1) 完成三點 → 結束
            if self.hit_index >= 3 and self.seq_phase == 'IDLE':
                #self.keyboard_pub.publish(String(data='e'))
                self.hit_sequence_active = False
                self.tile_index += 1
                self.get_logger().info(f'Hit sequence completed for tile {self.tile_index-1}')

            # (2) 發送下一個移動指令
            elif self.seq_phase == 'IDLE':
                ix, iy = self.target_points_img[self.hit_index]
                wx, wy = self.image_to_workspace(ix, iy)
                # 邊界檢查（超界不發布）
                if (self.workspace_x_min <= wx <= self.workspace_x_max) and (self.workspace_y_min <= wy <= self.workspace_y_max):
                    #self.coordinates_pub.publish(String(data=f"{wx:.2f},{wy:.2f}"))
                    self.get_logger().info(f'[MOVE] to point {self.hit_index+1}/3 -> {wx:.2f},{wy:.2f}')
                else:
                    self.get_logger().warn(f'[SKIP] point out of workspace: {wx:.2f},{wy:.2f}')
                self.seq_phase = 'MOVING'
                self.cmd_time = time.time()

            # (3) 等待 ready → 執行打擊
            elif self.seq_phase == 'MOVING':
                if self.last_status == 'ready' and (time.time() - self.cmd_time) >= self.min_move_settle:
                    #  self.keyboard_pub.publish(String(data='h'))
                    self.get_logger().info(f'[HIT] on point {self.hit_index+1}')
                    self.seq_phase = 'WAIT_HIT'
                    self.cmd_time = time.time()

            # (4) 打擊完成 → 進下一點
            elif self.seq_phase == 'WAIT_HIT':
                if self.last_status == 'ready' and (time.time() - self.cmd_time) >= self.min_hit_gap:
                    self.hit_index += 1
                    if self.hit_index < 3:
                        #self.keyboard_pub.publish(String(data='n'))
                        self.get_logger().info(f'[NEXT] advance to point {self.hit_index+1}')
                    self.seq_phase = 'IDLE'

        # ======================== 顯示畫面 ========================
        # 顯示影像（若在校正模式，覆蓋 Color 視窗為凍結畫面 + 引導）
        cv2.imshow('AutoHit Vision - Processed (Gray+Edges)', proc_vis)
        if self.calib_mode and self.calib_background is not None:
            vis = self.calib_background.copy()
            guide = [
                'Window frozen. Start calibration: click BL → BR → TR → TL',
                'Calibration: click 4 points in order',
                '1) Bottom-Left  2) Bottom-Right  3) Top-Right  4) Top-Left',
                f'Clicked: {len(self.calib_img_points)}/4',
                'Keys: c=start, r=reset H, l=load H, q=quit'
            ]
            y0 = 24
            for i, line in enumerate(guide):
                cv2.putText(vis, line, (10, y0 + i*22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            label_map = ['BL', 'BR', 'TR', 'TL']
            for i, (px, py) in enumerate(self.calib_img_points):
                cv2.circle(vis, (int(px), int(py)), 6, (0, 255, 255), -1)
                label = label_map[i] if i < len(label_map) else str(i+1)
                cv2.putText(vis, label, (int(px)+6, int(py)-6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow('AutoHit Vision - Color', vis)
        else:
            cv2.imshow('AutoHit Vision - Color', color_vis)

        # 鍵盤控制：q 退出；c 開始校正；r 清除 H；l 載入 H
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rclpy.shutdown()
        elif key == ord('c'):
            # 開始校正（凍結當前畫面）
            self.start_calibration(color_vis)
        elif key == ord('r'):
            self.homography_H = None
            self.get_logger().info('Homography reset; fallback to linear mapping')
        elif key == ord('l'):
            self.load_homography()


# ======================== 主程序入口 ========================
def main(args=None):
    rclpy.init(args=args)
    node = AutoHitVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"節點執行錯誤: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

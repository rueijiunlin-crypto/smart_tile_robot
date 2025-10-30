#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
#實驗用紙

class TileDetector(Node):
    def __init__(self):
        super().__init__('tile_detector')
        # 訂閱相機影像主題
        self.color_subscription = self.create_subscription(
            Image,
            '/tile_robot/D435i/color/image_raw',
            self.color_image_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/tile_robot/D435i/depth/image_rect_raw',
            self.depth_image_callback,
            10)

        # 設定座標的發布者
        self.world_coordinates_publisher = self.create_publisher(String, '/World_Coordinates', 10)

        # 初始化 OpenCV 與 CvBridge
        self.bridge = CvBridge()
        self.mouseX, self.mouseY = -1, -1
        self.depth_image = None

        # 建立 OpenCV 視窗，並綁定滑鼠回呼函數
        cv2.namedWindow('Tiles')
        cv2.setMouseCallback('Tiles', self.show_coordinates)

        # 相機內參矩陣與失真參數
        self.mtx = np.array([[615.700927734375, 0, 332.12557983],
                             [0, 615.8004150390625, 239.88970947265625],
                             [0, 0, 1]])
        self.dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # 相機在世界座標系中的位置
        self.camera_world_position = np.array([145.0, 0, 0.0])

    def show_coordinates(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouseX, self.mouseY = x, y

    def color_image_callback(self, msg):
        # 轉換 ROS Image 訊息為 OpenCV 圖像
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.depth_image is not None:
            self.process_image(image)

    def depth_image_callback(self, msg):
        # 轉換 ROS Image 訊息為 OpenCV 深度圖像
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def process_image(self, image):
        # # 調整圖像對比度與亮度
        # contrast = 100
        # brightness = 100
        # output = image * (contrast/127 + 1) - contrast + brightness #轉換公式
        # output = np.clip(output, 0, 255)
        # output = np.uint8(output)

        # #cv2.imshow('oxxostudio2', output)

        # hsv = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)

        # lower_bound = np.array([42, 0, 84])  
        # upper_bound = np.array([145, 142, 206])

        # # 創建遮罩來過濾指定範圍
        # mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # # 將篩選範圍以外的區域設為黑色
        # result = image.copy()
        # result[mask == 0] = [0, 0, 0]

        # #cv2.imshow("hsv Image", result)


        # # 腐蝕與膨脹操作以去除雜訊
        # erosion_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # erosion_kerne2 = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        # dilated2 = cv2.dilate(result, erosion_kernel, iterations=2)
        # #cv2.imshow('dilated2', dilated2)
        # eroded2 = cv2.erode(dilated2, erosion_kerne2, iterations=2)
        # #cv2.imshow('eroded', eroded2)


        # # 計算圖像中心點
        # image_center = (image.shape[1] // 2, image.shape[0] // 2)

        # # 邊緣檢測
        # edged = cv2.Canny(eroded2, 60, 150)

        # #cv2.imshow('Canny', edged)

        # # 閉運算平滑邊緣
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel, iterations=1)

        # # 找到輪廓
        # contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
       
        # # 創建空白圖像並填充輪廓
        # filled_image = np.zeros_like(closed)
        # cv2.drawContours(filled_image, contours, -1, (255, 255, 255), thickness=cv2.FILLED)
        
        
        # # 腐蝕與膨脹操作以去除雜訊
        # erosion_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        # eroded = cv2.erode(filled_image, erosion_kernel, iterations=5)
        # dilated = cv2.dilate(eroded, erosion_kernel, iterations=5)

        # # 在膨脹後的圖像上找輪廓
        # eroded_contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        area_threshold=35000
        #area_threshold_m=65000
        # # 顯示最終處理結果
        # #cv2.imshow('Processed Image', dilated)
        # # 儲存所有紅點座標
        # red_points = []
        # 將圖像轉為灰階並模糊處理
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 計算圖像中心點
        image_center = (image.shape[1] // 2, image.shape[0] // 2)

        # 邊緣檢測
        edged = cv2.Canny(blurred, 40, 150)

        # 閉運算平滑邊緣
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 找到輪廓
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 創建空白圖像並填充輪廓
        filled_image = np.zeros_like(closed)
        cv2.drawContours(filled_image, contours, -1, (255, 255, 255), thickness=cv2.FILLED)

        # 腐蝕與膨脹操作以去除雜訊
        erosion_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        eroded = cv2.erode(filled_image, erosion_kernel, iterations=10)
        dilated = cv2.dilate(eroded, erosion_kernel, iterations=10)

        # 在膨脹後的圖像上找輪廓
        eroded_contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 儲存所有紅點座標
        red_points = []


        for contour in eroded_contours:
            area = cv2.contourArea(contour)
            if area>area_threshold:
                if area > 2000:
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int32(box)
                    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)

                    center = (int(rect[0][0]), int(rect[0][1]))
                    red_points.append(center)

                    if area > 10000:
                        width, height = rect[1]
                        angle = rect[2]

                        if width < height:
                            width, height = height, width
                            angle += 90

                        radian = np.deg2rad(angle)
                        left_point = (int(center[0] - width / 4 * np.cos(radian)), int(center[1] - width / 4 * np.sin(radian)))
                        right_point = (int(center[0] + width / 4 * np.cos(radian)), int(center[1] + width / 4 * np.sin(radian)))

                        # 繪製中心點與左右點
                        cv2.circle(image, center, 5, (0, 0, 255), -1)
                        cv2.circle(image, left_point, 5, (0, 0, 255), -1)
                        cv2.circle(image, right_point, 5, (0, 0, 255), -1)
                        red_points.append(left_point)
                        red_points.append(right_point)
                    else:
                        cv2.circle(image, center, 5, (0, 0, 255), -1)

        # 如果偵測到紅點，計算世界座標
        if red_points:
            camera_3d_points = []
            fx = self.mtx[0, 0]
            fy = self.mtx[1, 1]
            cx = self.mtx[0, 2]
            cy = self.mtx[1, 2]

            for point in red_points:
                x_pixel, y_pixel = point
                if 0 <= x_pixel < self.depth_image.shape[1] and 0 <= y_pixel < self.depth_image.shape[0]:
                    depth_value = self.depth_image[y_pixel, x_pixel] * 1  # 深度值以毫米為單位
                else:
                    depth_value = 1.0  # 如果超出範圍，使用預設值

                x_camera = (x_pixel - cx) * depth_value / fx
                y_camera = (y_pixel - cy) * depth_value / fy
                camera_3d_points.append([x_camera, y_camera, depth_value])

            # 轉換為世界座標
            world_3d_points = []
            for cam_point in camera_3d_points:
                x_world = cam_point[0] + self.camera_world_position[0]
                y_world = -cam_point[1] + self.camera_world_position[1]  # y 軸翻轉
                z_world = cam_point[2]
                world_3d_points.append([x_world, y_world, z_world])

            # 格式化座標字串
            coordinates_str = ",".join([f"({p[0]:.2f},{p[1]:.2f})" for p in world_3d_points])

            # 建立訊息並發布
            msg = String()
            msg.data = coordinates_str
            self.world_coordinates_publisher.publish(msg)
            #self.get_logger().info(f"Published world coordinates: {msg.data}")

        # 顯示圖像
        #cv2.circle(image, image_center, 5, (0, 255, 255), -1)
        img_copy = image.copy()
        if self.mouseX >= 0 and self.mouseY >= 0:
            cv2.putText(img_copy, f"({self.mouseX}, {self.mouseY})", (self.mouseX, self.mouseY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.imshow('Tiles', img_copy)

        # 按 'q' 鍵退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tile_detector = TileDetector()
    rclpy.spin(tile_detector)

    # 關閉所有視窗
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
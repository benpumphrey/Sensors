import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from PyQt5.QtWidgets import (QApplication, QLabel, QWidget, QGridLayout, 
                             QVBoxLayout, QHBoxLayout, QFrame)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QImage, QPixmap, QFont

class ROSWorker(QThread):
    # Signals to send data from ROS thread to the UI thread
    image_signal = pyqtSignal(np.ndarray, str)
    coords_signal = pyqtSignal(float, float, float)

    def run(self):
        rclpy.init()
        self.node = Node('marty_dashboard_node')
        
        # Use Best Effort QoS to prevent lag at 240 FPS
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        # Subscribers matching your Launch File namespacing
        self.node.create_subscription(CompressedImage, '/camera/left/compressed', 
                                      self.left_callback, qos)
        self.node.create_subscription(CompressedImage, '/camera/right/compressed', 
                                      self.right_callback, qos)
        
        # Subscribing to your Stereo Node's 3D output
        self.node.create_subscription(Point, '/ball_tracker/3d_coords', 
                                      self.stereo_callback, qos)
        
        rclpy.spin(self.node)

    def left_callback(self, msg): self.process_frame(msg, "left")
    def right_callback(self, msg): self.process_frame(msg, "right")

    def stereo_callback(self, msg):
        self.coords_signal.emit(msg.x, msg.y, msg.z)

    def process_frame(self, msg, side):
        # Decode the JPEG stream
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self.image_signal.emit(frame, side)

class MartyDashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("M.A.R.T.Y. Control Center | 240 FPS System")
        self.setFixedSize(1400, 800)
        self.init_ui()
        
        # Start the background ROS thread
        self.worker = ROSWorker()
        self.worker.image_signal.connect(self.update_image)
        self.worker.coords_signal.connect(self.update_stats)
        self.worker.start()

    def init_ui(self):
        self.setStyleSheet("background-color: #1a1a1a; color: #e0e0e0;")
        main_layout = QVBoxLayout()

        # --- Top Section: Video Feeds ---
        video_layout = QHBoxLayout()
        
        self.left_feed = self.create_video_label("LEFT CAMERA (JETSON A)")
        self.right_feed = self.create_video_label("RIGHT CAMERA (JETSON B)")
        
        video_layout.addWidget(self.left_feed)
        video_layout.addWidget(self.right_feed)
        main_layout.addLayout(video_layout)

        # --- Bottom Section: Live Data ---
        data_frame = QFrame()
        data_frame.setStyleSheet("background-color: #2d2d2d; border-radius: 10px; border: 1px solid #444;")
        data_layout = QGridLayout(data_frame)

        title_font = QFont("Consolas", 14, QFont.Bold)
        val_font = QFont("Consolas", 24, QFont.Bold)

        # 3D Coordinate Display
        data_layout.addWidget(QLabel("3D BALL POSITION (METERS)"), 0, 0, 1, 3, Qt.AlignCenter)
        self.lbl_x = self.create_stat_label("X: 0.00", "#ff5555", val_font)
        self.lbl_y = self.create_stat_label("Y: 0.00", "#55ff55", val_font)
        self.lbl_z = self.create_stat_label("Z: 0.00", "#5555ff", val_font)

        data_layout.addWidget(self.lbl_x, 1, 0)
        data_layout.addWidget(self.lbl_y, 1, 1)
        data_layout.addWidget(self.lbl_z, 1, 2)

        main_layout.addWidget(data_frame)
        self.setLayout(main_layout)

    def create_video_label(self, text):
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("border: 2px solid #555; background-color: #000;")
        lbl.setMinimumSize(640, 400)
        return lbl

    def create_stat_label(self, text, color, font):
        lbl = QLabel(text)
        lbl.setFont(font)
        lbl.setStyleSheet(f"color: {color};")
        lbl.setAlignment(Qt.AlignCenter)
        return lbl

    def update_image(self, frame, side):
        # Convert OpenCV BGR to Qt RGB
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_img).scaled(640, 400, Qt.KeepAspectRatio)
        
        if side == "left":
            self.left_feed.setPixmap(pixmap)
        else:
            self.right_feed.setPixmap(pixmap)

    def update_stats(self, x, y, z):
        self.lbl_x.setText(f"X: {x:.2f}")
        self.lbl_y.setText(f"Y: {y:.2f}")
        self.lbl_z.setText(f"Z: {z:.2f}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MartyDashboard()
    window.show()
    sys.exit(app.exec_())
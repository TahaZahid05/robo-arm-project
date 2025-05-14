import sys
import serial
import time
import math
import cv2
import torch
import numpy as np
import requests
import glob
from PyQt5.QtCore import Qt, QTimer, QPoint
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QMessageBox
import pathlib

pathlib.PosixPath = pathlib.WindowsPath


arduino = serial.Serial(port='COM10', baudrate=9600, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

def calculate_thetas(matrix):
    # Given link lengths in cm
    L1 = 11.7
    L2 = 12.7
    L3 = 8.89
    L5 = 3.5

    # Extract matrix entries
    r = matrix

    # Calculate theta1 using r14 and r24
    theta1 = math.atan2(r[0][3], r[1][3])

    # Precompute sin and cos of theta1
    sin_theta1 = math.sin(theta1)
    cos_theta1 = math.cos(theta1)

    # Calculate theta5 using theta1, r11, r21, r12, r22
    numerator_theta5 = sin_theta1 * r[0][0] - cos_theta1 * r[1][0]
    denominator_theta5 = sin_theta1 * r[0][1] - cos_theta1 * r[1][1]
    theta5 = math.atan2(numerator_theta5, denominator_theta5)

    # Calculate theta2 using theta1 and theta5
    tan_theta5 = math.tan(theta5)
    term1_theta2_num = cos_theta1 * r[0][1] + sin_theta1 * r[1][1]
    term2_theta2_num = tan_theta5 * (cos_theta1 * r[0][0] + sin_theta1 * r[1][0])
    numerator_theta2 = -(term1_theta2_num + term2_theta2_num)
    denominator_theta2 = r[0][2] * tan_theta5 + r[2][1]
    theta2 = math.atan2(numerator_theta2, denominator_theta2)

    # Precompute sin and cos of theta2
    sin_theta2 = math.sin(theta2)
    cos_theta2 = math.cos(theta2)

    # Calculate rho for theta3
    term_rho1 = cos_theta1 * r[0][3] + sin_theta1 * r[1][3]
    term_rho2 = L1 - r[2][3]
    rho = cos_theta2 * term_rho1 - sin_theta2 * term_rho2 - L2

    # Calculate lambda for theta3
    lambda_ = -cos_theta2 * term_rho2 - sin_theta2 * term_rho1

    # Calculate beta for theta3
    term_beta = cos_theta1 * r[0][2] + sin_theta1 * r[1][2]
    beta = sin_theta2 * r[2][2] + cos_theta2 * term_beta

    # Calculate S for theta3
    term_S = cos_theta1 * r[0][2] + sin_theta1 * r[1][2]
    S = cos_theta2 * r[2][2] - sin_theta2 * term_S

    # Calculate theta3
    numerator_theta3 = (lambda_ - L5 * S) / L3
    denominator_theta3 = (rho - L5 * beta) / L3
    theta3 = math.atan2(numerator_theta3, denominator_theta3)

    # Calculate theta4
    term_theta4 = cos_theta1 * r[0][2] + sin_theta1 * r[1][2]
    theta4 = math.atan2(r[2][2], term_theta4) - theta3 - theta2

    # Convert all angles from radians to degrees
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)
    theta3_deg = math.degrees(theta3)
    theta4_deg = math.degrees(theta4)
    theta5_deg = math.degrees(theta5)

    return theta1_deg, theta2_deg, theta3_deg, theta4_deg, theta5_deg

class ObjectDetectionApp(QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up main window
        self.setWindowTitle("YOLOv5 Object Detection")
        self.setGeometry(100, 100, 800, 700)

        # QLabel to display video feed (640 x 640 video feed)
        self.video_label = QLabel(self)
        self.video_label.setGeometry(10, 10, 640, 640)
        self.video_label.setStyleSheet("border: 1px solid black")
        # selection_rect = rectangle drawn by the user
        # start_point = initial point from where user starts drawing
        # selection_made = Did the user select an object?
        self.selection_rect = None
        self.start_point = None
        self.selection_made = False

        # QLabel to display object info
        # Displays camera coordinates in the application
        self.info_label = QLabel(self)
        self.info_label.setGeometry(10, 650, 780, 50)
        self.info_label.setStyleSheet("border: 1px solid black")
        self.info_label.setAlignment(Qt.AlignLeft)

        # Timer for video feed
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)

        # Load YOLOv5 model
        try:
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load YOLOv5 model: {e}")
            sys.exit(1)

        # IP Camera URL
        self.image_url = "http://10.20.1.67:8080/shot.jpg"  # Replace with your IP camera URL

        self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = self.calibrate_camera("calibration_image")

        test_points = [
            (0, 0), (640, 0), (0, 640), (640, 640),
            (320, 320), (160, 320), (320, 160), (480, 320)
        ]

        for u, v in test_points:
            
            pixel_coords = np.array([u, v, 1])
            cam_coords = 24 * (np.linalg.inv(self.camera_matrix) @ pixel_coords)
            print(f"Pixel ({u}, {v}) -> Camera ({cam_coords[0]:.2f}, {cam_coords[1]:.2f}, 24)")


        # Timer for video feed
        self.timer.start(30)

        # Variables for detections and selection
        self.detections = []
        self.selected_object = None

    # def calibrate_camera(self, images_folder, pattern_size=(8, 6), square_size=0.0508):
    #     """
    #     Calibrate the camera using checkerboard images.
        
    #     :param images_folder: Path to folder containing checkerboard images.
    #     :param pattern_size: Number of internal corners in the checkerboard (cols, rows).
    #     :param square_size: Size of a square in your checkerboard (in meters).
    #     :return: Camera matrix, distortion coefficients, rotation and translation vectors.
    #     """
    #     # Prepare object points for the checkerboard
    #     objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    #     objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    #     objp *= square_size

    #     # Arrays to store object points and image points
    #     objpoints = []  # 3D points in real-world space
    #     imgpoints = []  # 2D points in image plane

    #     # Get all images in the folder
    #     images = glob.glob(f"{images_folder}/*.jpg")

    #     for fname in images:
    #         img = cv2.imread(fname)
    #         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #         # Find the chessboard corners
    #         ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    #         if ret:
    #             objpoints.append(objp)
    #             imgpoints.append(corners)

    #             # Draw and display the corners
    #             cv2.drawChessboardCorners(img, pattern_size, corners, ret)
    #             cv2.imshow('Checkerboard', img)
    #             cv2.waitKey(500)

    #     cv2.destroyAllWindows()

    #     # Calibrate the camera
    #     ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    #     if ret:
    #         print("Camera Matrix:\n", mtx)
    #         print("\nDistortion Coefficients:\n", dist)
    #         return mtx, dist, rvecs, tvecs
    #     else:
    #         print("Camera calibration failed.")
    #         return None, None, None, None

    def calibrate_camera(self,images_folder, checkerboard_size=(8, 6)):
        # Define the dimensions of the checkerboard
        CHECKERBOARD = checkerboard_size

        # Stop the iteration when specified accuracy is reached
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Vector for 3D points
        threedpoints = []

        # Vector for 2D points
        twodpoints = []

        # 3D points in real-world coordinates
        objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

        # Extract the path of individual images in the given directory
        images = glob.glob(f"{images_folder}/*.jpg")

        for filename in images:
            image = cv2.imread(filename)
            grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(grayColor, CHECKERBOARD,
                                                    cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                    cv2.CALIB_CB_FAST_CHECK +
                                                    cv2.CALIB_CB_NORMALIZE_IMAGE)

            # If corners are detected, refine the pixel coordinates and add to points
            if ret == True:
                print(f"Corners detected in {filename}")
                threedpoints.append(objectp3d)

                # Refining pixel coordinates for given 2D points
                corners2 = cv2.cornerSubPix(grayColor, corners, (11, 11), (-1, -1), criteria)

                twodpoints.append(corners2)

                # Draw and display the corners
                image = cv2.drawChessboardCorners(image, CHECKERBOARD, corners2, ret)
            else:
                print(f"No corners detected in {filename}")

            # Display the image with corners (optional)
            cv2.imshow('img', image)
            cv2.waitKey(0)

        cv2.destroyAllWindows()

        # Perform camera calibration
        h, w = image.shape[:2]
        ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(threedpoints, twodpoints, grayColor.shape[::-1], None, None)

        # Display the calibration results
        print("Camera matrix:")
        print(matrix)

        print("\nDistortion coefficients:")
        print(distortion)

        print("\nRotation Vectors:")
        print(r_vecs)

        print("\nTranslation Vectors:")
        print(t_vecs)

        return matrix, distortion, r_vecs, t_vecs

    def fetch_frame_from_ip_camera(self):
        try:
            # Fetch the image from the URL
            response = requests.get(self.image_url, stream=True)
            if not response.ok:
                self.info_label.setText("Error: Unable to fetch image from IP camera.")
                return None

            # Convert the image to a NumPy array
            image_array = np.array(bytearray(response.content), dtype=np.uint8)

            # Decode the image to OpenCV format
            frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

            # Resize the frame to 640x640
            frame_resized = cv2.resize(frame, (640, 640))

            # Undistort using original camera matrix (no scaling needed)
            undistorted_frame = cv2.undistort(frame_resized, self.camera_matrix, self.dist_coeffs)

            return undistorted_frame

        except Exception as e:
            self.info_label.setText(f"Error fetching IP camera frame: {e}")
            return None

    def update_frame(self):
        # Get the frame from the IP camera
        frame = self.fetch_frame_from_ip_camera()
        if frame is None:
            return

        # Draw selection rectangle if selected
        if self.selection_rect:
            x1, y1, x2, y2 = self.selection_rect
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Only perform YOLOv5 inference if a selection has been made
        if self.selection_made:
            x1, y1, x2, y2 = self.selection_rect
            cropped_frame = frame[y1:y2, x1:x2]

            if cropped_frame.size == 0 or cropped_frame.shape[0] == 0 or cropped_frame.shape[1] == 0:
                self.info_label.setText("Invalid selection area. Please select a valid region.")
                return

            try:
                results = self.model(cropped_frame)
                detections = results.pandas().xyxy[0]
                self.detections = []

                for _, row in detections.iterrows():
                    x1_crop, y1_crop, x2_crop, y2_crop = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                    x1_orig, y1_orig, x2_orig, y2_orig = x1 + x1_crop, y1 + y1_crop, x1 + x2_crop, y1 + y2_crop
                    self.detections.append({'bbox': (x1_orig, y1_orig, x2_orig, y2_orig), 'label': row['name'], 'confidence': row['confidence']})

                    label = f"{row['name']} {row['confidence']:.2f}"
                    cv2.rectangle(frame, (x1_orig, y1_orig), (x2_orig, y2_orig), (255, 0, 0), 2)
                    cv2.putText(frame, label, (x1_orig, y1_orig - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # if self.selected_object:
                #     x1_sel, y1_sel, x2_sel, y2_sel = self.selected_object['bbox']
                #     center_x = (x1_sel + x2_sel) // 2
                #     center_y = (y1_sel + y2_sel) // 2
                #     pixel_coords = np.array([center_x, center_y, 1])
                #     camera_coords = 24 * (np.linalg.inv(self.camera_matrix) @ pixel_coords)

                #     X = camera_coords[0]
                #     Y = camera_coords[1]
                #     Z = 24

                #     print(f"Camera coordinates: ({X:.2f}, {Y:.2f}, {Z:.2f})")
                #     label = self.selected_object['label']
                #     cv2.rectangle(frame, (x1_sel, y1_sel), (x2_sel, y2_sel), (0, 255, 255), 2)
                #     self.info_label.setText(f"Selected: {label} ({x1_sel}, {y1_sel}) to ({x2_sel}, {y2_sel})")
                if self.selected_object:
                    x1_sel, y1_sel, x2_sel, y2_sel = self.selected_object['bbox']
                    center_x = (x1_sel + x2_sel) // 2
                    center_y = (y1_sel + y2_sel) // 2
                    pixel_coords = np.array([center_x, center_y, 1])
                    camera_coords = 24 * (np.linalg.inv(self.camera_matrix) @ pixel_coords)

                    X = camera_coords[0]
                    Y = camera_coords[1]
                    Z = 24

                    label = self.selected_object['label']
                    cv2.rectangle(frame, (x1_sel, y1_sel), (x2_sel, y2_sel), (0, 255, 255), 2)

                    # Estimate 2D orientation
                    roi = frame[y1_sel:y2_sel, x1_sel:x2_sel]
                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    _, thresh = cv2.threshold(gray_roi, 50, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    orientation_matrix = None
                    angle_deg = None

                    if contours:
                        largest = max(contours, key=cv2.contourArea)
                        rect = cv2.minAreaRect(largest)
                        angle_deg = rect[2]

                        # Normalize angle if needed
                        width, height = rect[1]
                        if width < height:
                            theta = angle_deg
                        else:
                            theta = angle_deg + 90

                        theta_rad = math.radians(theta)

                        # Build 3x3 rotation matrix (Z-axis rotation)
                        R = np.array([
                            [math.cos(theta_rad), -math.sin(theta_rad), 0],
                            [math.sin(theta_rad),  math.cos(theta_rad), 0],
                            [0, 0, 1]
                        ])

                        # Build translation vector
                        t = np.array([[X], [Y], [Z]])

                        # Combine into 4x4 transformation matrix
                        T = np.vstack((np.hstack((R, t)), [0, 0, 0, 1]))

                        orientation_matrix = T

                        print("Transformation matrix (4x4):")
                        # print(T)
                        
                        deg1, deg2, deg3, deg4, deg5 = calculate_thetas(T)
                        arduino.write((deg1,",",deg2,",",deg3,",",deg4,",",deg5,"\n").encode())
                        
                        time.sleep(0.1)
                        while arduino.in_waiting:
                            response = arduino.readline().decode().strip()
                            print("From Arduino: " , response)
                        

                        self.info_label.setText(
                            f"{label}: Pos=({X:.2f}, {Y:.2f}, {Z}), θ={theta:.2f}°"
                        )
                    else:
                        self.info_label.setText(
                            f"{label}: Pos=({X:.2f}, {Y:.2f}, {Z}), Orientation=Unavailable"
                        )

            except Exception as e:
                self.info_label.setText(f"Inference error: {e}")
                return

        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.video_label.setPixmap(QPixmap.fromImage(qt_image))

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            label_pos = self.video_label.pos()
            frame_width = 640  # Width of the frame (resized to 640x640)
            frame_height = 640  # Height of the frame (resized to 640x640)
            label_width, label_height = self.video_label.width(), self.video_label.height()
            x, y = (event.x() - label_pos.x()) * frame_width // label_width, (event.y() - label_pos.y()) * frame_height // label_height
            if self.selection_made:
                for detection in self.detections:
                    x1, y1, x2, y2 = detection['bbox']
                    print(x,y,x1,y1,x2,y2)
                    if x1 <= x <= x2 and y1 <= y <= y2:
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        pixel_coords = np.array([center_x,center_y,24])
                        camera_coords = np.linalg.inv(self.camera_matrix) @ pixel_coords
                        X = camera_coords[0]
                        Y = camera_coords[1]
                        Z = 24
                        print(f"Camera coordinates: ({X:.2f}, {Y:.2f}, {Z:.2f})")
                        self.selected_object = detection
                        self.info_label.setText(f"Selected: {detection['label']} ({x1}, {y1}) to ({x2}, {y2})")
                        self.update_frame()  # Update the frame with yellow border on selected object
                        return
            self.start_point = QPoint(x, y)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.start_point:
            x, y = self.start_point.x(), self.start_point.y()
            x2, y2 = event.x(), event.y()
            self.selection_rect = (min(x, x2), min(y, y2), max(x, x2), max(y, y2))
            self.selection_made = True
            self.start_point = None
            self.update_frame()

        elif event.button() == Qt.RightButton:
            self.selection_rect = None
            self.selection_made = False
            self.selected_object = None
            self.info_label.setText("")

    def closeEvent(self, event):
        arduino.close()
        self.cap.release()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ObjectDetectionApp()
    window.show()
    sys.exit(app.exec_())

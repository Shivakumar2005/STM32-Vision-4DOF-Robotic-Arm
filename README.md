# Vision-Based 4-DOF Robotic Arm with Real-Time MATLAB Digital Twin & STM32 Hardware Control

A cyber-physical robotics system that translates natural hand gestures captured via a webcam into real-time joint angles. The calculated kinematics simultaneously drive a **MATLAB 3D Digital Twin** via UDP and a physical **4-DOF Robotic Arm** using an **STM32 Microcontroller** and a **PCA9685 16-Channel PWM Servo Driver**.

---

## System Architecture
<img width="2816" height="1536" alt="Gemini_Generated_Image_vv4mgvvv4mgvvv4m" src="https://github.com/user-attachments/assets/6443ac2c-56db-4d0d-a5b2-163e40d9d4d9" />


Mathematical Model & Kinematics

The system bypasses complex inverse kinematics by utilizing direct spatial mapping. We map specific geometric relationships from the 2D camera frame (W x H) to the angular constraints of the servo motors.

Let (x_c, y_c) be the center of the camera frame, defined as:
x_c = W / 2,  y_c = H / 2

1. Base Rotation (Theta 1)
Controlled by the absolute horizontal position (x) of the hand center (MediaPipe Landmark 9).
Theta_1 = (x / W) * 180°

2. Shoulder Flexion (Theta 2)
Controlled by the absolute vertical position (y) of the hand center. The mapping is inverted so that raising the hand raises the arm.
Theta_2 = 120° - ((y / H) * 120°)

3. Elbow Extension (Theta 3)
Controlled by the Euclidean distance (d_center) between the hand center and the camera frame center. As the hand moves away from the center of the screen, the arm extends.
d_center = √((x - x_c)² + (y - y_c)²)
Theta_3 = (d_center / d_max) * 90°
(where d_max is a predefined maximum threshold radius).

4. End-Effector Gripper (Theta 4)
Controlled by the pinch distance (d_pinch) between the tip of the index finger (L_8) and the tip of the thumb (L_4).
d_pinch = √((x_8 - x_4)² + (y_8 - y_4)²)
Theta_4 = map(d_pinch, [20, 150] -> [0°, 90°])
# OUTPUT IMAGES
<img width="1915" height="1143" alt="image" src="https://github.com/user-attachments/assets/1fb67567-a708-42a6-843e-f5ebcb211dba" />
<img width="801" height="644" alt="image" src="https://github.com/user-attachments/assets/45a781b0-84eb-4a70-8e4e-0bf7df8c3a1e" />
<img width="900" height="1600" alt="STM32_CONNECTION " src="https://github.com/user-attachments/assets/3554d3fd-ce7f-46e3-a23f-8c534b515fdf" />



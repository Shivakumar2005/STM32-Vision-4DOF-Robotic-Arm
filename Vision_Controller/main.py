import cv2
import mediapipe as mp
import serial
import socket
import math
import numpy as np

# ----------------- MATLAB UDP SETUP -----------------
UDP_IP = "127.0.0.1"
UDP_PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print("UDP Socket ready for MATLAB Digital Twin...")

# ----------------- STM32 SERIAL SETUP ---------------
# Change 'COM3' to your STM32's actual COM port
try:
    ser = serial.Serial('COM29', 115200, timeout=1) 
    print("Serial port opened successfully! Ready for Physical Robot...")
except Exception as e:
    print(f"Error opening serial port: {e}")
    print("Physical robot will not move. MATLAB simulation will still run.")

# -------------- MEDIAPIPE SETUP --------------
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

while True:
    success, img = cap.read()
    if not success:
        print("Failed to grab frame")
        break
        
    img = cv2.flip(img, 1)
    h, w, c = img.shape
    
    center_x, center_y = int(w / 2), int(h / 2)
    
    # Draw White Crosshair
    cv2.line(img, (center_x, 0), (center_x, h), (255, 255, 255), 1) 
    cv2.line(img, (0, center_y), (w, center_y), (255, 255, 255), 1) 
    
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(img, handLms, mp_hands.HAND_CONNECTIONS)

            lm_list = []
            for id, lm in enumerate(handLms.landmark):
                cx, cy = int(lm.x * w), int(lm.y * h)
                lm_list.append([cx, cy])

            if len(lm_list) != 0:
                hand_x, hand_y = lm_list[9][0], lm_list[9][1]
                
                # --- CALCULATE KINEMATICS ---
                theta1 = np.interp(hand_x, [0, w], [0, 180])
                theta2 = np.interp(hand_y, [0, h], [120, 0])
                
                dx = hand_x - center_x
                dy = hand_y - center_y
                diagonal_dist = math.hypot(dx, dy)
                max_dist = math.hypot(center_x, center_y) / 1.5 
                theta3 = np.interp(diagonal_dist, [0, max_dist], [0, 90])

                thumb_x, thumb_y = lm_list[4][0], lm_list[4][1]
                index_x, index_y = lm_list[8][0], lm_list[8][1]
                pinch_dist = math.hypot(index_x - thumb_x, index_y - thumb_y)
                theta4 = np.interp(pinch_dist, [20, 150], [0, 90])

                # Draw Visualizers
                cv2.line(img, (center_x, center_y), (hand_x, hand_y), (255, 0, 255), 2)
                cv2.line(img, (thumb_x, thumb_y), (index_x, index_y), (0, 255, 255), 2)
                cv2.circle(img, (hand_x, hand_y), 8, (0, 0, 255), cv2.FILLED)

                # --- 1. TRANSMIT TO MATLAB (UDP) ---
                msg_udp = f"{theta1:.2f},{theta2:.2f},{theta3:.2f},{theta4:.2f}"
                sock.sendto(msg_udp.encode(), (UDP_IP, UDP_PORT))

                # --- 2. TRANSMIT TO STM32 (Serial) ---
                # Serial requires the \n character to trigger the STM32 interrupt
                msg_serial = f"{theta1:.2f},{theta2:.2f},{theta3:.2f},{theta4:.2f}\n"
                try:
                    ser.write(msg_serial.encode())
                except NameError:
                    pass 
                
                cv2.putText(img, f"T1(Base): {int(theta1)}  T2(Shoulder): {int(theta2)}", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(img, f"T3(Elbow): {int(theta3)} T4(Gripper): {int(theta4)}", 
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Vision-Based Digital Twin & STM32 Controller", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
try:
    ser.close()
except:
    pass
cv2.destroyAllWindows()

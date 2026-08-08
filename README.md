# Vision-Based 4-DOF Robotic Arm with Real-Time MATLAB Digital Twin & STM32 Hardware Control

**Project Title**
Vision-Based 4-DOF Robotic Arm with Real-Time MATLAB Digital Twin & STM32 Hardware Control

**Objective**
To develop a cyber-physical robotics system that translates natural hand gestures captured via a webcam into real-time joint angles. The system continuously maps kinematics to simultaneously drive a MATLAB 3D Digital Twin and a physical 4-DOF robotic arm utilizing an STM32 microcontroller and a dedicated PWM driver.

### 1. System Overview
The Vision-Based 4-DOF Robotic Arm is an advanced cyber-physical solution built around the STM32F446RE Nucleo Board. The system combines real-time computer vision, digital twin simulation, and embedded hardware control to create an intuitive, touchless interface.
Users control the arm using hand gestures tracked by a Python script utilizing MediaPipe and OpenCV. When hand movements are processed, the system calculates the inverse kinematics and spatial mappings. The resulting joint angles are simultaneously broadcast over UDP to update a MATLAB 3D simulation and over UART to the STM32. The STM32 offloads the physical actuation to a PCA9685 driver via I2C, moving the base, shoulder, elbow, and gripper smoothly and without jitter.

### 2. Hardware Components
| Component | Purpose |
| :--- | :--- |
| STM32F446RE Nucleo Board | Main Controller |
| PCA9685 16-Channel PWM Driver | Servo Motor Control |
| Servo Motors ×4 | Robotic Arm Joints (Base, Shoulder, Elbow, Gripper) |
| Webcam | Hand Tracking Input |
| Li-ion Battery & Buck Converter | Power Supply |
| Breadboard | Circuit Prototyping |
| Jumper Wires | Electrical Connections |
| USB Cable | Power & Programming |

### 3. Software Used
| Software | Purpose |
| :--- | :--- |
| STM32CubeIDE | Firmware Development |
| STM32CubeMX | Peripheral Configuration |
| Embedded C | Programming Language |
| STM32 HAL Drivers | Hardware Abstraction |
| Python (OpenCV, MediaPipe) | Vision Controller & Kinematics |
| MATLAB | 3D Digital Twin Simulation |

### 4. Working Principle
The system operates through parallel computer vision tracking and dual-channel communication for simulation and hardware actuation.

**Hand Tracking & Kinematics**
The webcam continuously monitors the user's hand.
Using MediaPipe:
• Absolute X position controls the Base Rotation.
• Absolute Y position controls the Shoulder Flexion (inverted).
• Distance from the frame center controls Elbow Extension.
• Thumb-to-index pinch distance controls the Gripper.

**Data Transmission & Hardware Actuation**
Once calculated, the joint angles are transmitted:
• Over UDP to MATLAB for instant 3D rendering.
• Over UART to the STM32.
The STM32 utilizes non-blocking circular interrupts to receive the data stream without freezing. It then formats and sends precise I2C commands to the PCA9685 to adjust the PWM signals driving the servos.

### 5. System Workflow
1. Power ON the STM32 and the external power supply for the servos.
2. Launch the MATLAB Digital Twin to listen on UDP port 5000.
3. Run the Python Vision Controller script.
4. Read webcam frames and track hand landmarks.
5. Calculate the targeted joint angles based on spatial mapping.
6. Transmit data simultaneously via UDP and UART.
7. MATLAB renders the 3D robotic arm model in real-time.
8. STM32 receives and parses the UART data stream via interrupts.
9. STM32 sends I2C commands to the PCA9685 driver.
10. Actuate the physical servo motors to mirror the digital twin and the user's hand.

### 6. System Features
• Touchless gestural interface
• Real-time spatial kinematic mapping
• Simultaneous digital twin and physical actuation
• High-speed UDP local network socket communication
• Non-blocking UART circular receive interrupts
• Hardware-offloaded PWM generation via I2C
• 3D MATLAB simulation for visual validation
• Modular architecture separating vision, simulation, and embedded control

### 7. Software Architecture
The embedded firmware is developed using Embedded C in STM32CubeIDE with the STM32 HAL Library. The PC environment utilizes Python for vision/networking and MATLAB for simulation.

The embedded software consists of the following modules:
• GPIO Initialization
• UART Communication (Interrupt-driven)
• I2C Interface
• PCA9685 Driver Integration
• String Parsing and Angle Conversion
• Main Application Loop

**Peripheral Usage**
| Peripheral | Function |
| :--- | :--- |
| GPIO | Status LEDs |
| I2C1 (PB8, PB9) | Communication with PCA9685 |
| USART2 | UART Communication with Python Script |
| HAL Library | Hardware Abstraction |

The main loop continuously:
1. Waits for UART circular buffer updates.
2. Parses the received string to extract four joint angles.
3. Converts the angles into appropriate PWM pulse lengths.
4. Transmits the pulse lengths to the PCA9685 via I2C.
5. Updates the physical position of the servos.

### 8. Advantages
• Intuitive and natural human-machine interface
• Safe algorithm validation through the MATLAB digital twin prior to hardware execution
• Jitter-free and precise motor movement due to PCA9685 offloading
• Microcontroller is protected from processing bottlenecks via interrupt-based architecture
• Low-cost embedded implementation for complex robotics
• Easily expandable for additional degrees of freedom or sensor integration

### 9. Images

<img width="2816" height="1536" alt="Gemini_Generated_Image_vv4mgvvv4mgvvv4m" src="https://github.com/user-attachments/assets/b7e3151f-4ab2-4f38-9c23-24a0c6f2230b" />

<img width="801" height="644" alt="image" src="https://github.com/user-attachments/assets/36b27f5d-58b7-4b9f-991e-81889d79fd8e" />

<img width="1915" height="1143" alt="image" src="https://github.com/user-attachments/assets/eda2be9c-4b0a-4833-961b-8751fdea5dc5" />

<img width="900" height="1600" alt="STM32_CONNECTION " src="https://github.com/user-attachments/assets/e395be46-0f52-4198-a667-0a04484ef933" />



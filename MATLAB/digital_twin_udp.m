% 4-DOF Robotic Arm Digital Twin with Real-Time UDP Updating
clear; clc; close all;

% 1. Build the 4-DOF Rigid Body Tree
robot = rigidBodyTree('DataFormat', 'row');

% Joint 1: Base Rotation (Yaw - Z axis)
body1 = rigidBody('base_link');
jnt1 = rigidBodyJoint('jnt1', 'revolute');
jnt1.JointAxis = [0 0 1]; 
jnt1.PositionLimits = deg2rad([0 180]); 
setFixedTransform(jnt1, trvec2tform([0 0 0.05]));
body1.Joint = jnt1;
addBody(robot, body1, 'base');

% Joint 2: Shoulder (Pitch - Y axis)
body2 = rigidBody('shoulder_link');
jnt2 = rigidBodyJoint('jnt2', 'revolute');
jnt2.JointAxis = [0 1 0];
jnt2.PositionLimits = deg2rad([0 120]); 
setFixedTransform(jnt2, trvec2tform([0 0 0.15]));
body2.Joint = jnt2;
addBody(robot, body2, 'base_link');

% Joint 3: Elbow (Pitch - Y axis)
body3 = rigidBody('elbow_link');
jnt3 = rigidBodyJoint('jnt3', 'revolute');
jnt3.JointAxis = [0 1 0];
jnt3.PositionLimits = deg2rad([0 90]); 
setFixedTransform(jnt3, trvec2tform([0 0 0.15]));
body3.Joint = jnt3;
addBody(robot, body3, 'shoulder_link');

% Joint 4: Wrist/Gripper (Pitch - Y axis)
body4 = rigidBody('gripper_link');
jnt4 = rigidBodyJoint('jnt4', 'revolute');
jnt4.JointAxis = [0 1 0];
setFixedTransform(jnt4, trvec2tform([0 0 0.1]));
body4.Joint = jnt4;
addBody(robot, body4, 'elbow_link');

% 2. Initialize Visualization
figure('Name', 'Vision-Controlled 4-DOF Digital Twin', 'NumberTitle', 'off');
show(robot);
xlim([-0.4 0.4]); ylim([-0.4 0.4]); zlim([0 0.6]);
view(45, 30); grid on;
title('Waiting for MediaPipe Gesture Data...');

% 3. UDP Setup & Real-Time Control Loop
prev_angles = [90, 60, 45, 0]; 
% Turn Frames 'on' so the joint axes are visible
show(robot, deg2rad(prev_angles), 'PreservePlot', false, 'Frames', 'on');
drawnow; 

u = udpport("LocalPort", 5000);
disp('UDP Server active. Listening on port 5000...');

while true
    if u.NumBytesAvailable > 0
        dataStr = char(read(u, u.NumBytesAvailable, "char"));
        raw_angles = str2double(split(dataStr, ','))';
        
        if length(raw_angles) == 4
            
            % --- THE INVERSION FIX ---
            % Invert the 2nd DOF (Shoulder) to match physical hand movement.
            % Limit is 120 degrees, so subtract from 120.
            raw_angles(2) = 120.0 - raw_angles(2);
            
            % Exponential Moving Average (EMA) Smoothing
            filtered_angles = (0.8 * prev_angles) + (0.2 * raw_angles);
            prev_angles = filtered_angles;
            
            q = deg2rad(filtered_angles);
            
            % Turn Frames 'on' during the real-time update loop
            show(robot, q, 'PreservePlot', false, 'FastUpdate', true, 'Frames', 'on');
            drawnow;
        end
    else
        pause(0.01); 
    end
end

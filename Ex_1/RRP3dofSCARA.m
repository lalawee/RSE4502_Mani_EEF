% MATLAB code to create a 3-DOF manipulator with (revolute, revolute, prismatic) joints

% Clear workspace and close all figures
clear;
clc;
close all;

% Create a rigidBodyTree object
robot = rigidBodyTree('DataFormat', 'column');

% Define the lengths of the links (for visualization purposes)
L0 = 0.1; % Length of Base link
L1 = 0.2; % Length of the first link
L2 = 0.15; % Length of the second link

% The default base link is named 'base' in a rigidBody tree config

% Create the first revolute joint (Joint 1)
body1 = rigidBody('body1');
joint1 = rigidBodyJoint('joint1', 'revolute');
joint1.HomePosition = 0; % Home position of the joint
setFixedTransform(joint1, trvec2tform([0 0 (L0/2+0.01)])); % Set the transform for the joint
body1.Joint = joint1;
addBody(robot, body1, 'base'); % Add the first body to the base

% Create the second revolute joint (Joint 2)
body2 = rigidBody('body2');
joint2 = rigidBodyJoint('joint2', 'revolute');
joint2.HomePosition = 0; % Home position of the joint
setFixedTransform(joint2, trvec2tform([L1 0 0])); % Set the transform for the joint
body2.Joint = joint2;
addBody(robot, body2, 'body1'); % Add the second body to the first body

% Create the prismatic link
body3 = rigidBody('body3');
setFixedTransform(body3.Joint, trvec2tform([L2 0 0])); % No additional transform
addBody(robot, body3, 'body2'); % Add the third body to the second body

% Add an end effector and prismatic joint(Joint 3)
endEffector = rigidBody('endEffector');
joint3 = rigidBodyJoint('joint3', 'prismatic');
joint3.HomePosition = 0; % Home position of the joint
setFixedTransform(joint3, trvec2tform([0 0 0])); % Set the transform for the joint
endEffector.Joint = joint3;
addBody(robot, endEffector, 'body3');

% Display the robot
show(robot);
title('3-DOF Manipulator (Revolute-Revolute-Prismatic)');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on;

% Set random joint configurations for visualization
config = randomConfiguration(robot);
show(robot, config);

%Display
showdetails(robot)
iGUI = interactiveRigidBodyTree(robot,"MarkerScaleFactor",1.0);
function updateRobotConfiguration(robot, jointAngles)
% updateRobotConfiguration
% Updates joint positions and visualizes the robot.
%
% jointAngles must be in radians and match the number of non-fixed joints.

config = homeConfiguration(robot);

% Update only as many joints as provided (usually 5 for JetArm)
for i = 1:min(length(config), length(jointAngles))
    config(i).JointPosition = jointAngles(i);
end

show(robot, config, "Frames", "on");
title(['JetArm - Joint Angles (deg): ', mat2str(rad2deg(jointAngles), 3)]);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; axis equal;
end

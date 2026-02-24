path = defineWeldPath();

% Check position range
disp('X range:'); disp([min(path.pos(1,:)), max(path.pos(1,:))]);
disp('Y range:'); disp([min(path.pos(2,:)), max(path.pos(2,:))]);
disp('Z:');       disp(unique(path.pos(3,:)));

% Plot
figure;
plot3(path.pos(1,:), path.pos(2,:), path.pos(3,:), 'r-', 'LineWidth', 2);
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Welding Path in Robot Base Frame');
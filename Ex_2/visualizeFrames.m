function visualizeFrames(T)
    % visualizeFrames: Visualizes the world frame and a transformed frame.
    % Input:
    %   T: 4x4 homogeneous transformation matrix representing the transformed frame.
    %
    % Usage:
    %   Change view angle with the command view(azimuth,elevation)
    %   azimuth, elevation are specified in degrees
    %   For sideview: view(0,0)
    %   For topdownview: view(0,90)
    %   For obliqueview: view(30,30) etc

    % Validate input
    if ~isequal(size(T), [4, 4])
        error('Input must be a 4x4 homogeneous transformation matrix.');
    end

    % Extract the rotation matrix and translation vector
    R = T(1:3, 1:3); % Rotation matrix
    P = T(1:3, 4);   % Translation vector

    % Define the origin and axes of the world frame
    origin = [0; 0; 0]; % Origin of the world frame
    x_axis = [1; 0; 0]; % X-axis of the world frame
    y_axis = [0; 1; 0]; % Y-axis of the world frame
    z_axis = [0; 0; 1]; % Z-axis of the world frame

    % Transform the axes using the rotation matrix and translation vector
    x_axis_transformed = R * x_axis + P;
    y_axis_transformed = R * y_axis + P;
    z_axis_transformed = R * z_axis + P;

    % Plot the world frame and transformed frame
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Visualization of World Frame and Transformed Frame');

    % Plot world frame
    quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 2); % X-axis
    quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 2); % Y-axis
    quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 2); % Z-axis

    % Plot transformed frame
    %quiver3(P(1), P(2), P(3), x_axis_transformed(1) - P(1), x_axis_transformed(2) - P(2), x_axis_transformed(3) - P(3), 'r', 'LineWidth', 2); % X-axis
    %quiver3(P(1), P(2), P(3), y_axis_transformed(1) - P(1), y_axis_transformed(2) - P(2), y_axis_transformed(3) - P(3), 'g', 'LineWidth', 2); % Y-axis
    %quiver3(P(1), P(2), P(3), z_axis_transformed(1) - P(1), z_axis_transformed(2) - P(2), z_axis_transformed(3) - P(3), 'b', 'LineWidth', 2); % Z-axis
    quiver3(P(1),P(2),P(3), ...
            x_axis_transformed(1)-P(1), ...
            x_axis_transformed(2)-P(2), ...
            x_axis_transformed(3)-P(3), ...
            'Color', [0.6 0.3 0.1], 'LineWidth', 2);   % X'

    
    quiver3(P(1),P(2),P(3), y_axis_transformed(1)-P(1), ...
            y_axis_transformed(2)-P(2), y_axis_transformed(3)-P(3), ...
            'm', 'LineWidth', 2);   % Y'
    
    quiver3(P(1),P(2),P(3), z_axis_transformed(1)-P(1), ...
            z_axis_transformed(2)-P(2), z_axis_transformed(3)-P(3), ...
            'y', 'LineWidth', 2);   % Z'



    % Add legend and annotations
    legend('World X', 'World Y', 'World Z', 'Transformed X', 'Transformed Y', 'Transformed Z');
    legend('location','bestoutside')
    text(P(1), P(2), P(3), 'Transformed Frame', 'Color', 'k', 'FontSize', 12);
    text(origin(1), origin(2), origin(3), 'World Frame', 'Color', 'k', 'FontSize', 12);

    % Set plot limits for better visualization
    xlim([-1 4]);
    ylim([-1 4]);
    zlim([-1 4]);

    hold off;
end
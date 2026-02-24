function plotFrame(T, name, color)
    % Plots a coordinate frame given its transformation matrix
    origin = T(1:3, 4);
    R = T(1:3, 1:3);
    
    % Axis vectors (scaled for visibility)
    scale = 0.8;
    x_axis = R * [scale; 0; 0];
    y_axis = R * [0; scale; 0];
    z_axis = R * [0; 0; scale];
    
    % Plot arrows
    quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), color, 'LineWidth', 2, 'DisplayName', [name ' X']);
    quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), color, 'LineWidth', 2, 'DisplayName', [name ' Y']);
    quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), color, 'LineWidth', 2, 'DisplayName', [name ' Z']);
    
    % Label
    text(origin(1), origin(2), origin(3) + 0.3, ['Frame ' name], 'FontSize', 10, 'Color', color);
end

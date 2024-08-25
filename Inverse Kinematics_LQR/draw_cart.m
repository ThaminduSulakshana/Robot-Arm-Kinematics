function draw_cart(state)
    % DRAW_CART Visualizes the cart-pendulum system.
    % Input: 
    %   state - A vector containing the current state [x, v, theta, omega]
    
    % Dimensions
    W = 1; % Cart width (meters)
    H = 0.5; % Cart height (meters)
    wr = 0.2; % Wheel radius (meters)
    mr = 0.3; % Pendulum mass radius (meters)
    L = 2; % Length of the pendulum (meters)

    % Extract positions from the state vector
    x = state(1); % Cart position (meters)
    theta = state(3); % Pendulum angle (radians)
    y = wr/2 + H/2; % Vertical position of the cart center (meters)

    % Wheel positions (x, y coordinates)
    w1x = x - 0.9*W/2; % X-coordinate of the first wheel
    w1y = 0; % Y-coordinate of the first wheel
    w2x = x + 0.9*W/2 - wr; % X-coordinate of the second wheel
    w2y = 0; % Y-coordinate of the second wheel

    % Pendulum end position (x, y coordinates)
    px = x + L*sin(theta); % X-coordinate of the pendulum end
    py = y + L*cos(theta); % Y-coordinate of the pendulum end

    % Plotting the ground line
    plot([-10 10], [0 0], 'k', 'LineWidth', 1.5); % Horizontal ground line
    hold on;

    % Plotting the cart
    rectangle('Position', [x-W/2, y-H/2, W, H], 'Curvature', 0.1, 'FaceColor', [0.5 0.5 1], 'LineWidth', 1);
    % Cart represented as a rectangle with curved corners

    % Plotting the wheels
    rectangle('Position', [w1x, w1y, wr, wr], 'Curvature', 1, 'FaceColor', [0 0 0], 'LineWidth', 1);
    % First wheel represented as a filled circle
    rectangle('Position', [w2x, w2y, wr, wr], 'Curvature', 1, 'FaceColor', [0 0 0], 'LineWidth', 1);
    % Second wheel represented as a filled circle

    % Plotting the pendulum
    plot([x px], [y py], 'k', 'LineWidth', 1.5); % Line representing the pendulum

    % Plotting the pendulum mass
    rectangle('Position', [px-mr/2, py-mr/2, mr, mr], 'Curvature', 1, 'FaceColor', [1 0.1 0.1], 'LineWidth', 1);
    % Pendulum mass represented as a filled circle

    % Set the plot limits
    xlim([-5 5]); % X-axis limits
    ylim([-3 3]); % Y-axis limits

    % Update the plot
    drawnow;
    hold off;
end

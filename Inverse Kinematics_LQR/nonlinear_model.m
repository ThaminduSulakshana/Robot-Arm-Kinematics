function dx = nonlinear_model(x, L, M, m, g, d, u)
    % NONLINEAR_MODEL Computes the derivatives of the state vector for the cart-pendulum system.
    % This function calculates the time derivatives of the cart position, cart velocity,
    % pendulum angle, and pendulum angular velocity.
    % Inputs:
    %   x - State vector [x, v, theta, omega]
    %     x(1) = x - Cart position (meters)
    %     x(2) = v - Cart velocity (meters/second)
    %     x(3) = theta - Pendulum angle (radians)
    %     x(4) = omega - Pendulum angular velocity (radians/second)
    %   L - Length of the pendulum (meters)
    %   M - Mass of the cart (kg)
    %   m - Mass of the pendulum (kg)
    %   g - Acceleration due to gravity (m/s^2)
    %   d - Damping coefficient (N·s/m)
    %   u - Control input (force applied to the cart)
    % Output:
    %   dx - Derivative of the state vector [dx/dt, dv/dt, dtheta/dt, domega/dt]
    
    % Extract state variables from the input vector
    x1 = x(1); % Cart position
    x2 = x(2); % Cart velocity
    x3 = x(3); % Pendulum angle
    x4 = x(4); % Pendulum angular velocity
    
    % Initialize the derivative vector
    dx = zeros(4,1);
    
    % dx(1) = x2 - The rate of change of cart position is the cart velocity
    dx(1,1) = x2;
    
    % dx(2) - The acceleration of the cart is derived from the system dynamics
    % Formula accounts for the forces due to the pendulum, control input, damping, and gravity
    dx(2,1) = (m*L*x4^2*sin(x3) + u - m*g*sin(x3)*cos(x3) - d*x2) / (M + m - m*cos(x3)^2);
    
    % dx(3) = x4 - The rate of change of pendulum angle is the pendulum angular velocity
    dx(3,1) = x4;
    
    % dx(4) - The angular acceleration of the pendulum
    % Formula considers the effects of the control input, pendulum dynamics, damping, and gravity
    dx(4,1) = -((u + m*L*x4^2*sin(x3) - d*x2)*cos(x3) - (M + m)*g*sin(x3)) / (L*(M + m - m*cos(x3)^2));
end

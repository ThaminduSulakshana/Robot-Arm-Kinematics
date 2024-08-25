clear;  % Clear all variables from the workspace
close all;  % Close all figure windows
clc;  % Clear the command window

%% Parameters
L = 2;      % Length of the pendulum (meters)
M = 5;      % Mass of the cart (kg)
m = 1;      % Mass of the pendulum (kg)
g = 9.8;    % Acceleration due to gravity (m/s^2)
d = 1;      % Damping coefficient (N·s/m)

%% Modeling
% Define the state-space representation of the system
A = [0 1 0 0;   % State matrix for the linearized system
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 d/(M*L) (m+M)*g/(M*L) 0];
B = [0; 1/M; 0; -1/(M*L)];  % Input matrix for control input

% Check the controllability of the system
controllable = rank(ctrb(A,B));  % Compute the controllability matrix and its rank
disp(['Controllability matrix rank: ', num2str(controllable)]);

% Compute the eigenvalues of the system matrix A to analyze stability
eigen = eig(A);  % Eigenvalues of the state matrix
disp('Eigenvalues of A:');
disp(eigen);

%% Controller design
% Define the weighting matrices for the Linear Quadratic Regulator (LQR)
Q = diag([1, 1, 1, 10]);  % State weighting matrix, emphasizing the importance of angular position
R = 0.001;  % Control input weighting scalar, affecting the control effort

% Compute the optimal LQR gain matrix
K = lqr(A,B,Q,R);  % Calculate the LQR gain matrix for state feedback control
disp('LQR Gain Matrix K:');
disp(K);

%% Simulation
% Define the time span for the simulation
tspan = 0:0.01:8;  % Time vector from 0 to 8 seconds with a step of 0.01 seconds

% Define the initial state vector [x0, v0, theta0, omega0]
x0 = [-2; 0; 0.1; 0];   % Initial conditions: [cart position, cart velocity, pendulum angle, pendulum angular velocity]
wr = [2; 0; 0; 0];     % Reference state (desired state): [desired cart position, desired velocity, desired angle, desired angular velocity]

% Define the control law as an anonymous function
u = @(x) -K*(x - wr);  % Compute the control input using the LQR gain matrix and state error

% Simulate the system using the ode45 solver
[t, x] = ode45(@(t,x) nonlinear_model(x,L,M,m,g,d,u(x)), tspan, x0, odeset);

% Plot the results
for k = 1:length(t)
    draw_cart(x(k,:));  % Function to visualize the cart and pendulum at each time step (assumed to be defined elsewhere)
end

%% Plot results
figure;  % Create a new figure window for the plots
plot(t, x, 'LineWidth', 2);  % Plot the state variables over time with line width of 2
hold on;  % Retain current plot when adding new plots
legend('x', 'v', '\theta', '\omega');  % Add a legend to the plot
xlabel('Time');  % X-axis label
ylabel('State');  % Y-axis label
grid on;  % Enable grid on the plot for better visualization

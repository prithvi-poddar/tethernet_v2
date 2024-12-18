clc; clear all; close all;
% function simulate_moving_system()
% Time span for simulation
tspan = [0, 10]; % Start and end time
dt = 0.1; % Time step for visualization

% Initial conditions: [x_com, x_com_dot, y_com, y_com_dot, phi, phi_dot]
L = 5;
z0 = [3, 0, 0, 0, 0, 0]; % Initial positions and velocities

% Solve ODE
[t, z] = ode45(@system_dynamics, tspan, z0);

% Extract positions of the green circles
x_com = z(:,1); 
y_com = z(:,3);
phi = z(:,5);

% x1 = z(:, 1); y1 = z(:, 3); % Circle 1
% x2 = z(:, 5); y2 = z(:, 7); % Circle 2
x1 = x_com - L/2*cos(phi);
y1 = y_com - L/2*sin(phi);

x2 = x_com + L/2*cos(phi);
y2 = y_com + L/2*sin(phi);

dist = sqrt((x1-x2).^2 + (y1-y2).^2);

% Parameters for stationary blue circle
x0 = 0; y0 = 0; % Center of the stationary circle
r0 = 0.5; % Radius of the stationary circle

% Set up the figure for visualization
figure('Position', [10, 10, 800, 800]);
hold on;
axis equal;
axis([-10 20 -10 20]); % Set axis limits
grid on;

% Stationary circle (plotted once)
theta = linspace(0, 2 * pi, 100);
fill(x0 + r0 * cos(theta), y0 + r0 * sin(theta), 'b', 'FaceAlpha', 0.5);

% Initialize dynamic elements
h_circle1 = plot(x1(1), y1(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
h_circle2 = plot(x2(1), y2(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
h_rod = plot([x1(1), x2(1)], [y1(1), y2(1)], 'b-', 'LineWidth', 2);

% Animate the motion
for i = 1:length(t)
    % Update positions of green circles
    set(h_circle1, 'XData', x1(i), 'YData', y1(i));
    set(h_circle2, 'XData', x2(i), 'YData', y2(i));

    % Update the connecting rod
    set(h_rod, 'XData', [x1(i), x2(i)], 'YData', [y1(i), y2(i)]);

    % Pause for visualization
    pause(dt);
end

hold off;
% end

function dzdt = system_dynamics(t, z)
    % Parameters
    F1 = 5; % Force on MU1
    F2 = 2; % Force on MU2
    theta1 = pi/4; % Angle of F1
    theta2 = pi/3; % Angle of F2
    m1 = 5;        % Mass of MU1
    m2 = 5;        % Mass of MU2
    L = 5;

    % State variables
    x_com = z(1); x_com_dot = z(2);
    y_com = z(3); y_com_dot = z(4);
    phi = z(5); phi_dot = z(6);

    F1x = F1*cos(theta1-phi);
    F1y = -F1*sin(theta1-phi);
    F2x = F2*cos(theta2-phi);
    F2y = -F2*sin(theta2-phi);

    % Accelerations
    x_com_ddot = (F1x + F2x)/(m1 + m2);
    y_com_ddot = (F1y + F2y)/(m1 + m2);
    phi_ddot = 2/(m1*L)*((F2y-F1y)*cos(phi) - (F2x-F1x)*sin(phi));

    % Pack derivatives into dzdt
    dzdt = [x_com_dot; x_com_ddot; y_com_dot; y_com_ddot; phi_dot; phi_ddot];
end



% % function simulate_moving_system()
% % Time span for simulation
% tspan = [0, 10]; % Start and end time
% dt = 0.1; % Time step for visualization
% 
% % Initial conditions: [x1, x1_dot, y1, y1_dot, x2, x2_dot, y2, y2_dot]
% z0 = [0, 0, 0, 0, 2, 0, 1, 0]; % Initial positions and velocities
% 
% % Solve ODE
% [t, z] = ode45(@system_dynamics, tspan, z0);
% 
% % Extract positions of the green circles
% x1 = z(:, 1); y1 = z(:, 3); % Circle 1
% x2 = z(:, 5); y2 = z(:, 7); % Circle 2
% 
% % Parameters for stationary blue circle
% x0 = 0; y0 = 0; % Center of the stationary circle
% r0 = 0.5; % Radius of the stationary circle
% 
% % Set up the figure for visualization
% figure('Position', [10, 10, 800, 800]);
% hold on;
% axis equal;
% axis([-10 10 -10 10]); % Set axis limits
% grid on;
% 
% % Stationary circle (plotted once)
% theta = linspace(0, 2 * pi, 100);
% fill(x0 + r0 * cos(theta), y0 + r0 * sin(theta), 'b', 'FaceAlpha', 0.5);
% 
% % Initialize dynamic elements
% h_circle1 = plot(x1(1), y1(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% h_circle2 = plot(x2(1), y2(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% h_rod = plot([x1(1), x2(1)], [y1(1), y2(1)], 'b-', 'LineWidth', 2);
% 
% % Animate the motion
% for i = 1:length(t)
%     % Update positions of green circles
%     set(h_circle1, 'XData', x1(i), 'YData', y1(i));
%     set(h_circle2, 'XData', x2(i), 'YData', y2(i));
% 
%     % Update the connecting rod
%     set(h_rod, 'XData', [x1(i), x2(i)], 'YData', [y1(i), y2(i)]);
% 
%     % Pause for visualization
%     pause(dt);
% end
% 
% hold off;
% % end
% 
% function dzdt = system_dynamics(t, z)
%     % Parameters
%     F1 = 5; % Force on MU1
%     F2 = 2; % Force on MU2
%     theta1 = pi/4; % Angle of F1
%     theta2 = pi/3; % Angle of F2
%     phi = pi/6;    % Angle of the rod
%     m1 = 5;        % Mass of MU1
%     m2 = 7;        % Mass of MU2
% 
%     % State variables
%     x1 = z(1); x1_dot = z(2);
%     y1 = z(3); y1_dot = z(4);
%     x2 = z(5); x2_dot = z(6);
%     y2 = z(7); y2_dot = z(8);
% 
%     % Accelerations
%     x1_ddot = F1 * cos(theta1 - phi) / m1;
%     y1_ddot = F1 * sin(theta1 - phi) / m1;
%     x2_ddot = F2 * cos(theta2 - phi) / m2;
%     y2_ddot = F2 * sin(theta2 - phi) / m2;
% 
%     % Pack derivatives into dzdt
%     dzdt = [x1_dot; x1_ddot; y1_dot; y1_ddot; x2_dot; x2_ddot; y2_dot; y2_ddot];
% end

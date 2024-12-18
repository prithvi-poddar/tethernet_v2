clf; clear all; close all;
%% plotting

x1_opt = readmatrix('x1_opt.txt');
x2_opt = readmatrix('x2_opt.txt');
y1_opt = readmatrix('y1_opt.txt');
y2_opt = readmatrix('y2_opt.txt');
f1_opt = readmatrix('f1_opt.txt');
f2_opt = readmatrix('f2_opt.txt');
theta1_opt = readmatrix('theta1_opt.txt');
theta2_opt = readmatrix('theta2_opt.txt');
time = readmatrix('time.txt');

xo = 0;
yo = 0;
ro = 5.5;
L = 22;

x_com = (x1_opt + x2_opt)/2;
y_com = (y1_opt + y2_opt)/2;

figure(1); % Open a new figure window for the first plot
plot(time, x_com, 'r', 'LineWidth', 1.5);
hold on;
plot(time, y_com, 'b', 'LineWidth', 1.5);
title('Tracking the COM of the net');
xlabel('time');
ylabel('position');
legend('x_{com}', 'y_{com}', 'Location', 'best');
hold off;
grid on;

figure(2); % Open a new figure window for the first plot
plot(time, f1_opt, 'r', 'LineWidth', 1.5);
hold on;
plot(time, f2_opt, 'b', 'LineWidth', 1.5);
title('Thrust vs. Time');
xlabel('time');
ylabel('position');
legend('F_1', 'F_2', 'Location', 'best');
hold off;
grid on;

figure(3); % Open a new figure window for the first plot
plot(time, theta1_opt, 'r', 'LineWidth', 1.5);
hold on;
plot(time, theta2_opt, 'b', 'LineWidth', 1.5);
title('Thrust Angle vs. Time');
xlabel('time');
ylabel('position');
legend('\theta_1', '\theta_2', 'Location', 'best');
hold off;
grid on;


%% Simulate



tspan = [0, 10]; % Start and end time
dt = 0.1; % Time step for visualization

% Extract positions of the green circles
x1 = x1_opt; % z(:, 1); 
y1 = y1_opt; % z(:, 3); % Circle 1
x2 = x2_opt; % z(:, 5); 
y2 = y2_opt; % z(:, 7); % Circle 2

% Parameters for stationary blue circle
x0 = 0; y0 = 0; % Center of the stationary circle
r0 = 0.5; % Radius of the stationary circle

% Set up the figure for visualization
figure('Position', [10, 10, 800, 800]);
hold on;
axis equal;
axis([-50 50 -50 50]); % Set axis limits
grid on;

% Stationary circle (plotted once)
theta = linspace(0, 2 * pi, 100);
fill(xo + ro * cos(theta), yo + ro * sin(theta), 'b', 'FaceAlpha', 0.5);

% Initialize dynamic elements
h_circle1 = plot(x1(1), y1(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
h_circle2 = plot(x2(1), y2(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
h_rod = plot([x1(1), x2(1)], [y1(1), y2(1)], 'b-', 'LineWidth', 2);

trail_points = [];

% Animate the motion
for i = 1:length(x1)

    % Update positions of green circles
    set(h_circle1, 'XData', x1(i), 'YData', y1(i));
    set(h_circle2, 'XData', x2(i), 'YData', y2(i));
    
    % Update the connecting rod
    set(h_rod, 'XData', [x1(i), x2(i)], 'YData', [y1(i), y2(i)]);
    
    % Pause for visualization
    pause(dt);
end

hold off;


% 
% disp(['The total cost is: ', num2str(fopt)])
% 
% [c, ceq] = nonlcon(xopt, time, q_1_0, q_1_dot_0, q_2_0, q_2_dot_0, q_1_f, q_1_dot_f, q_2_f, q_2_dot_f);

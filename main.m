clc; clear all; close all;

%% Defining the constants
t0 = 0;
tf = 1;
n = 101; %number of collocation points

time = linspace(t0, tf, n);

xo = 0;
yo = 0;
ro = 5.5;
L = 22;

%% boundary conditions
x_com_0 = 30;
x_com_dot_0 = 0;
y_com_0 = 0;
y_com_dot_0 = 0;
phi_0 = 0;
phi_dot_0 = 0;


%% initial U, X guess
% initial U 
f1_init = zeros(1,n);
f2_init = zeros(1,n);
theta1_init = zeros(1,n)+deg2rad(90);
theta2_init = zeros(1,n)+deg2rad(90);

% initial X
x_com_init = ones(1, n-1) * x_com_0;
x_com_dot_init = ones(1, n-1) * x_com_dot_0;
y_com_init = ones(1, n-1) * y_com_0;
y_com_dot_init = ones(1, n-1) * y_com_dot_0;
phi_init = ones(1, n-1) * phi_0;
phi_dot_init = ones(1, n-1) * phi_dot_0;

z_init = 10;


% 0 <= theta <= pi
% 0 <= F <= 5
% TODO: try guessing tf only once instead of n times

X0 = [z_init, f1_init(1), f2_init(1), theta1_init(1), theta2_init(1)];

lb = [1, 0, 0, deg2rad(5), deg2rad(5)];
ub= [20, 5, 5, deg2rad(355), deg2rad(355)];

for i = 1:length(x_com_init)
    X0 = [X0, [x_com_init(i), x_com_dot_init(i), y_com_init(i), y_com_dot_init(i), phi_init(i), phi_dot_init(i), f1_init(i+1), f2_init(i+1), theta1_init(i+1), theta2_init(i+1)]];
    lb = [lb, [-50, -1000, -50, -1000, -2*pi, -20*pi, 0, 0, deg2rad(5), deg2rad(5)]];
    ub = [ub, [50, 1000, 50, 1000, 2*pi, 20*pi, 5, 5, deg2rad(355), deg2rad(355)]];
end


%% optimizing

options = optimoptions("fmincon", ...
    "Algorithm","interior-point", ...
    "EnableFeasibilityMode",true, ...
    "SubproblemAlgorithm","cg", ...
    "Display","iter", ...
    "ConstraintTolerance",1e-15, ...
    'MaxFunctionEvaluations',500000000, ...
    'UseParallel',true, ...
    'StepTolerance', 0, ...
    'MaxIterations', 2000);
% options = optimset('Display','iter','DerivativeCheck','off','GradConstr','off','GradObj','off', 'MaxF');

% opts = optimoptions('fmincon', ...
%     'OptimalityTolerance', 0, ...
%     'StepTolerance', 0, ...
%     'MaxFunctionEvaluations', inf,...
%     'MaxIterations', 1000);

[xopt, fopt] = fmincon(@obj,X0,[],[],[],[],lb,ub,@nonlcon,options,time, x_com_0, x_com_dot_0, y_com_0, y_com_dot_0, phi_0, phi_dot_0, xo, yo, ro, L);

%% plotting

x_com_opt = [x_com_dot_0];
y_com_opt = [y_com_0];
phi_opt = [phi_0];
z_opt = xopt(1);
f1_opt = [xopt(2)];
f2_opt = [xopt(3)];
theta1_opt = [xopt(4)];
theta2_opt = [xopt(5)];

% x_list = [x1_0, x1_dot_0, y1_0, y1_dot_0, x2_0, x2_dot_0, y2_0, y2_dot_0, phi_0 , phi_dot_0];
% u_list = [xopt(1), xopt(2), xopt(3), xopt(4)];

n_vars = length(xopt);

i = 6;
while i <= n_vars
    x_com_opt = [x_com_opt, xopt(i)];
    y_com_opt = [y_com_opt, xopt(i+2)];
    phi_opt = [phi_opt, xopt(i+4)];
    f1_opt = [f1_opt, xopt(i+6)];
    f2_opt = [f2_opt, xopt(i+7)];
    theta1_opt = [theta1_opt, xopt(i+8)];
    theta2_opt = [theta2_opt, xopt(i+9)];
    % u_list = [u_list; [xopt(i+10), xopt(i+11), xopt(i+12), xopt(i+13)]];
    i = i + 10;
end

x1_opt = x_com_opt - L/2*cos(phi_opt);
y1_opt = y_com_opt - L/2*sin(phi_opt);

x2_opt = x_com_opt + L/2*cos(phi_opt);
y2_opt = y_com_opt + L/2*sin(phi_opt);

% x_com = (x1_opt + x2_opt)/2;
% y_com = (y1_opt + y2_opt)/2;


figure;
subplot(2, 2, 1);
plot(time*z_opt, x_com_opt, 'r', 'LineWidth', 1.5);
hold on;
plot(time*z_opt, y_com_opt, 'b', 'LineWidth', 1.5);
tf =z_opt;
xline(tf, '--k', 'LineWidth', 1.5); 
text(tf - 0.1, 1, sprintf('t_f = %.2f s', tf), 'FontSize', 20, 'Color', 'k', 'HorizontalAlignment', 'right');
title('Tracking the COM of the net', 'FontSize', 15);
xlabel('time', 'FontSize', 30);
ylabel('position', 'FontSize', 30);
ax = gca; 
ax.FontSize = 20;
legend('x_{com}', 'y_{com}', 't_f', 'Location', 'best', 'FontSize', 20);
hold off;
grid on;

subplot(2, 2, 2);
plot(time*z_opt, f1_opt, 'r', 'LineWidth', 1.5);
hold on;
plot(time*z_opt, f2_opt, 'b', 'LineWidth', 1.5);
tf =z_opt;
xline(tf, '--k', 'LineWidth', 1.5); 
% text(tf - 0.1, 0, sprintf('t_f = %.2f s', tf), 'FontSize', 20, 'Color', 'k', 'HorizontalAlignment', 'right');
title('Thrust Magnitude vs. Time', 'FontSize', 15);
xlabel('time', 'FontSize', 30);
ylabel('Force (N)', 'FontSize', 30);
ax = gca; 
ax.FontSize = 20;
legend('F_1', 'F_2', 't_f', 'Location', 'best', 'FontSize', 20);
hold off;
grid on;

subplot(2, 2, 3);
plot(time*z_opt, rad2deg(theta1_opt), 'r', 'LineWidth', 1.5);
hold on;
plot(time*z_opt, rad2deg(theta2_opt), 'b', 'LineWidth', 1.5);
tf =z_opt;
xline(tf, '--k', 'LineWidth', 1.5); 
% text(tf - 0.1, 0, sprintf('t_f = %.2f s', tf), 'FontSize', 20, 'Color', 'k', 'HorizontalAlignment', 'right');
title('Thrust Angle vs. Time', 'FontSize', 15);
xlabel('time', 'FontSize', 30);
ylabel('Angle (degrees)', 'FontSize', 30);
ax = gca; 
ax.FontSize = 20;
legend('\theta_1', '\theta_2', 't_f', 'Location', 'best', 'FontSize', 20);
hold off;
grid on;

subplot(2, 2, 4);
x1 = x1_opt; 
y1 = y1_opt; 
x2 = x2_opt; 
y2 = y2_opt;
hold on;
axis equal;
% axis([-50 50 -50 50]); % Set axis limits
grid on;

% Stationary circle (plotted once)
theta = linspace(0, 2 * pi, 100);
fill(xo + ro * cos(theta), yo + ro * sin(theta), 'b', 'FaceAlpha', 0.5);


% Animate the motion
for i = 1:length(x1)

    if mod(i, 10) == 0
        plot([x1(i), x2(i)], [y1(i), y2(i)], 'b-', 'LineWidth', 2); 
        scatter([x1(i), x2(i)], [y1(i), y2(i)], 100, 'g', 'filled');
    end
end
ax = gca; 
ax.FontSize = 20;
title("Trajectory of the tether-net", 'FontSize', 20);

hold off;

%% Simulate

dt = 0.1; % Time step for visualization

% Extract positions of the green circles
x1 = x1_opt; % z(:, 1); 
y1 = y1_opt; % z(:, 3); % Circle 1
x2 = x2_opt; % z(:, 5); 
y2 = y2_opt; % z(:, 7); % Circle 2

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

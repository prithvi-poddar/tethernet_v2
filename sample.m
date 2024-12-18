% Initialize figure
figure;
hold on; grid on;

% Parameters
numSteps = 100; % Number of simulation steps
x0 = 0; y0 = 0; % Obstacle center (stationary)
r = 5; % Radius of obstacle

% Initialize ball positions
x1 = zeros(1, numSteps); y1 = zeros(1, numSteps);
x2 = zeros(1, numSteps); y2 = zeros(1, numSteps);

% Set initial positions of the two balls
x1(1) = -5; y1(1) = 30;
x2(1) = 5;  y2(1) = 30;

% Trajectory (example movement for both balls)
for k = 2:numSteps
    x1(k) = x1(k-1) + 0.5; % Ball 1 moves right
    y1(k) = y1(k-1) - 0.1; % Ball 1 moves down slightly
    
    x2(k) = x2(k-1) + 0.6; % Ball 2 moves right slightly faster
    y2(k) = y2(k-1) - 0.1; % Ball 2 moves down slightly
end

% Plot obstacle
viscircles([x0, y0], r, 'Color', 'b', 'LineWidth', 2);
fill(x0 + r*cos(linspace(0,2*pi,100)), y0 + r*sin(linspace(0,2*pi,100)), 'b', 'FaceAlpha', 0.5);

% Plot trajectories
for k = 1:numSteps
    if mod(k, 10) == 0
        % Plot the dashed wireframe of the rod
        plot([x1(k), x2(k)], [y1(k), y2(k)], 'b--', 'LineWidth', 0.5); % Dashed line
        scatter([x1(k), x2(k)], [y1(k), y2(k)], 100, 'g', 'filled'); % Green balls
    end
end

% Plot final position of the rod (solid)
plot([x1(end), x2(end)], [y1(end), y2(end)], 'b-', 'LineWidth', 2); % Solid line
scatter([x1(end), x2(end)], [y1(end), y2(end)], 100, 'g', 'filled'); % Green balls

% Adjust axis
axis equal;
xlim([-50 50]);
ylim([-50 50]);

% Labels
xlabel('X');
ylabel('Y');
title('Rod Trajectory with Wireframe History');

hold off;

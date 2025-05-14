% Author - Laxmi Nag L N

clc; clear; close all;

% Link lengths
L1 = 10;
L2 = 8;
z_up = 5;
z_down = 1;

% Hole positions (fixed in global frame)
holes = [10 0; 12 0; 14 0];

% Supply point (fixed in global frame)
supply = [5 -5];

% Store data for plotting
position_log = [];

figure;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-5 20]); ylim([-10 10]); zlim([0 8]);
view([120 30]);
grid on;
title('SCARA Robot Pick and Screw Simulation');

theta_base_deg = 0;

for i = 1:size(holes, 1)
    prev_theta_base = deg2rad(theta_base_deg);
    theta_base_deg = 10 * (i-1);
    curr_theta_base = deg2rad(theta_base_deg);

    % Animate base rotation
    for theta_base = linspace(prev_theta_base, curr_theta_base, 20)
        [theta1, theta2] = computeIK(supply(1), supply(2), L1, L2, theta_base);
        plotRobot(theta_base, theta1, theta2, z_up, L1, L2, holes, i, supply, 'Rotating base');
        pause(0.05);
    end

    theta_base = curr_theta_base;

    %% ----- (A) Move to Supply -----
    [theta1, theta2] = computeIK(supply(1), supply(2), L1, L2, theta_base);

    for z = z_up:-0.5:z_down
        plotRobot(theta_base, theta1, theta2, z, L1, L2, holes, i, supply, 'Picking screw');
        position_log(end+1,:) = [theta_base, theta1, theta2, z];
        pause(0.1);
    end
    pause(0.5);

    for z = z_down:0.5:z_up
        plotRobot(theta_base, theta1, theta2, z, L1, L2, holes, i, supply, 'Raising screw');
        position_log(end+1,:) = [theta_base, theta1, theta2, z];
        pause(0.1);
    end

    %% ----- (B) Move to Hole -----
    [theta1, theta2] = computeIK(holes(i,1), holes(i,2), L1, L2, theta_base);

    for z = z_up:-0.5:z_down
        plotRobot(theta_base, theta1, theta2, z, L1, L2, holes, i, supply, sprintf('Screwing Hole %d', i));
        position_log(end+1,:) = [theta_base, theta1, theta2, z];
        pause(0.1);
    end
    pause(1);

    for z = z_down:0.5:z_up
        plotRobot(theta_base, theta1, theta2, z, L1, L2, holes, i, supply, sprintf('Finished Hole %d', i));
        position_log(end+1,:) = [theta_base, theta1, theta2, z];
        pause(0.1);
    end
end

%% ========== Kinematic Analysis Plots ==========
dt = 0.1;
time = (0:size(position_log,1)-1) * dt;

base = rad2deg(position_log(:,1));
theta1 = rad2deg(position_log(:,2));
theta2 = rad2deg(position_log(:,3));
z = position_log(:,4);

omega_base = [0; diff(base)/dt];
omega1 = [0; diff(theta1)/dt];
omega2 = [0; diff(theta2)/dt];
vz = [0; diff(z)/dt];

alpha_base = [0; diff(omega_base)/dt];
alpha1 = [0; diff(omega1)/dt];
alpha2 = [0; diff(omega2)/dt];
az = [0; diff(vz)/dt];

figure('Name','Joint Positions vs Time');
plot(time, base, 'k', time, theta1, 'r', time, theta2, 'b', time, z*10, 'g');
legend('\theta_{base} (deg)','\theta_1 (deg)','\theta_2 (deg)','Z (cm)');
xlabel('Time (s)'); ylabel('Position'); title('Joint Positions'); grid on;

figure('Name','Joint Velocities vs Time');
plot(time, omega_base, 'k', time, omega1, 'r', time, omega2, 'b', time, vz*10, 'g');
legend('\omega_{base} (deg/s)','\omega_1 (deg/s)','\omega_2 (deg/s)','V_z (cm/s)');
xlabel('Time (s)'); ylabel('Velocity'); title('Joint Velocities'); grid on;

figure('Name','Joint Accelerations vs Time');
plot(time, alpha_base, 'k', time, alpha1, 'r', time, alpha2, 'b', time, az*10, 'g');
legend('\alpha_{base} (deg/s^2)','\alpha_1 (deg/s^2)','\alpha_2 (deg/s^2)','A_z (cm/s^2)');
xlabel('Time (s)'); ylabel('Acceleration'); title('Joint Accelerations'); grid on;

%% --- Inverse Kinematics ---
function [theta1, theta2] = computeIK(x, y, L1, L2, theta_base)
    R = [cos(-theta_base), -sin(-theta_base); sin(-theta_base), cos(-theta_base)];
    local = R * [x; y];
    xL = local(1);
    yL = local(2);
    r = sqrt(xL^2 + yL^2);
    cos_theta2 = (r^2 - L1^2 - L2^2)/(2*L1*L2);
    theta2 = acos(cos_theta2);
    k1 = L1 + L2*cos(theta2);
    k2 = L2*sin(theta2);
    theta1 = atan2(yL, xL) - atan2(k2, k1);
end

%% --- Plotting Function ---
function plotRobot(theta_base, theta1, theta2, z, L1, L2, holes, activeHole, supply, labelText)
    cla;
    base = [0; 0; 0];
    lift = [0; 0; z];

    p1 = [L1*cos(theta1); L1*sin(theta1)];
    p1_world = [cos(theta_base), -sin(theta_base); sin(theta_base), cos(theta_base)] * p1;
    joint1 = [p1_world; z];

    p2 = [L1*cos(theta1) + L2*cos(theta1 + theta2); L1*sin(theta1) + L2*sin(theta1 + theta2)];
    p2_world = [cos(theta_base), -sin(theta_base); sin(theta_base), cos(theta_base)] * p2;
    joint2 = [p2_world; z];

    plot3([base(1), base(1)], [base(2), base(2)], [0 z], 'k', 'LineWidth', 2); hold on;
    plot3([lift(1), joint1(1)], [lift(2), joint1(2)], [lift(3), joint1(3)], 'b', 'LineWidth', 3);
    plot3([joint1(1), joint2(1)], [joint1(2), joint2(2)], [joint1(3), joint2(3)], 'r', 'LineWidth', 3);
    plot3(joint1(1), joint1(2), joint1(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    plot3(joint2(1), joint2(2), joint2(3), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    plot3(joint2(1), joint2(2), joint2(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    for j = 1:size(holes, 1)
        hole = holes(j,:);
        color = 'g';
        if j == activeHole
            color = 'm';
        end
        plot3(hole(1), hole(2), 0, [color 'o'], 'MarkerSize', 8, 'MarkerFaceColor', color);
    end

    plot3(supply(1), supply(2), 0, 'cs', 'MarkerSize', 10, 'MarkerFaceColor', 'c');
    text(supply(1)+0.5, supply(2), 0.2, 'Supply', 'Color', 'c');
    text(joint2(1), joint2(2), joint2(3)+0.5, labelText, 'FontSize', 10);

    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-5 20]); ylim([-10 10]); zlim([0 8]);
    view([120 30]);
    grid on;
end
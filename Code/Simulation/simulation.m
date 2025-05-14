% Author - Laxmi Nag L N
% FINAL Coordinated Pick-and-Screw Simulation for Robot Arm
clear; clc; close all;

% Set up figure
figure;
hold on;
axis equal;
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Pick and Screw Operation (Fully Synchronized)');

% Pivot points
pivot_base = [65.17, 0.39, 112.45];
pivot1 = [139.07, -0.5, 363.8];
pivot2 = [198.07,  0.0, 331.0];

% Create transform hierarchy
t_base = hgtransform;
t_z = hgtransform('Parent', t_base);
t_middle = hgtransform('Parent', t_z);
t_gripper1 = hgtransform('Parent', t_middle);
t_arm1 = hgtransform('Parent', t_gripper1);
t_gripper2 = hgtransform('Parent', t_arm1);
t_arm2 = hgtransform('Parent', t_gripper2);
t_static = hgtransform;

% Attach STL files
attach_stl = @(filename, parent) ...
    patch('Faces', stlread(filename).ConnectivityList, ...
          'Vertices', stlread(filename).Points, ...
          'FaceColor', [0.8 0.8 1.0], ...
          'EdgeColor', 'none', ...
          'Parent', parent);

attach_stl('base_structure.stl', t_static);
attach_stl('base_plate.stl', t_base);
attach_stl('rods.stl', t_base);
attach_stl('top_plate.stl', t_base);
attach_stl('middle_plate.stl', t_middle);
attach_stl('gripper_link_1.stl', t_gripper1);
attach_stl('arm1.stl', t_arm1);
attach_stl('gripper_link2.stl', t_gripper2);
attach_stl('arm_2.stl', t_arm2);
attach_stl('end_effector.stl', t_arm2);

camlight; lighting gouraud;

%% === Master Sequence ===

% Define total keyframes per phase
N = 60;

% Full sequence with 6 segments
sequence = {
    % Base      Z-lift   Arm1   Arm2     (start → end over N steps)
    linspace(0, -90, N),  zeros(1,N),     linspace(20, 60, N),  linspace(-30, -80, N);
    -90*ones(1,N),        linspace(0, -90, N), linspace(60, 65, N), linspace(-80, -85, N);
    -90*ones(1,N),        linspace(-90, 0, N), linspace(65, 60, N), linspace(-85, -80, N);
    linspace(-90, 0, N),  zeros(1,N),     linspace(60, 20, N),  linspace(-80, -30, N);
    zeros(1,N),           linspace(0, -90, N), linspace(20, 35, N), linspace(-30, -45, N);
    zeros(1,N),           linspace(-90, 0, N), linspace(35, 20, N), linspace(-45, -30, N)
};

% Loop through each motion phase
for phase = 1:size(sequence,1)
    base_seq = sequence{phase,1};
    z_seq    = sequence{phase,2};
    arm1_seq = sequence{phase,3};
    arm2_seq = sequence{phase,4};

    for i = 1:N
        base_angle = deg2rad(base_seq(i));
        z_lift     = z_seq(i);
        arm1_angle = deg2rad(arm1_seq(i));
        arm2_angle = deg2rad(arm2_seq(i));

        % Apply transforms in sync
        t_base.Matrix = makehgtform('translate', pivot_base) * ...
                        makehgtform('zrotate', base_angle) * ...
                        makehgtform('translate', -pivot_base);

        t_z.Matrix = makehgtform('translate', [0 0 z_lift]);

        t_arm1.Matrix = makehgtform('translate', pivot1) * ...
                        makehgtform('zrotate', arm1_angle) * ...
                        makehgtform('translate', -pivot1);

        t_arm2.Matrix = makehgtform('translate', pivot2) * ...
                        makehgtform('zrotate', arm2_angle) * ...
                        makehgtform('translate', -pivot2);

        drawnow; pause(0.01);
    end
end

function[vehicle_param, rad_, num_, loc_matrix,...
    free_slide_matrix, wheel_speed, drive_dir, joint_angles,...
    joint_speed, joint_loc, joint_num, T, joint_lim,...
    wheel_angle]=PresetValues(cube_init, cube_fin)

% Wheel to wheel length and width in meters
vehicle_param =[0.47 0.3];

% Wheel radius, Number of wheels
rad_ = 0.0475;   
num_ = 4;

% create matrix to store wheel locations, driving direction, 
% speed wheel angle and free sliding direction 
loc_matrix = [[0.235 0.15]; [0.235 -0.15]; [-0.235 -0.15]; [-0.235 0.15]];
free_slide_matrix = [-0.785 0.785 -0.785 0.785];
wheel_speed = [0 0 0 0];
wheel_angle = zeros([1, num_]); 
drive_dir = zeros([num_, 1]);

% Number of revolute joints
joint_num = 5;

% create matrix to store joint angles, speed and locations
joint_angles = [0 0 0 0 0];
joint_speed = [0 0 0 0 0];
joint_loc = [[0 0 0]; [0.033 0 0.147]; [0.033 0 0.302]; [0.033 0 0.437];...
    [0.033 0 0.6546]];

% Fixed parameters - Trajectory.
% Trajectory generation parameters.
% Base

% T_sc_init -> Initial cube configuration.
T_sc_init = [[cos(cube_init(1)) -sin(cube_init(1)) 0;...
    sin(cube_init(1)) cos(cube_init(1)) 0;...
    0 0 1; 0 0 0],[cube_init(2:end) 0.025 1]'];

% T_sc_fin -> Final cube configuration.
T_sc_fin = [[cos(cube_fin(1)) -sin(cube_fin(1)) 0;...
    sin(cube_fin(1)) cos(cube_fin(1)) 0;...
    0 0 1; 0 0 0],[cube_fin(2:end) 0.025 1]'];

% Grasp angle orientation [Randomized]
a = 0;
b = pi;
r = (b-a).*rand(1,1) + a;

% T_ce_grab -> End effector position during object grip position.
T_ce_grab = [[-sin(r),0,-cos(r),0]',[0,1,0,0]',[cos(r),0,-sin(r),0]',...
    [0,0,0,1]'];

%T_ce_standoff.
T_ce_standoff = [[-0.7071, 0 , 0.7071, 0];[0,1,0,0];[-0.7071, 0, -0.7071, 0.06];[0,0,0,1]];

% T grouping.
T = {T_sc_init,T_sc_fin,T_ce_grab, T_ce_standoff};

% maximum joint limits on joints.
joint_lim = [[-2.932 2.932]; [-1.2 1.3]; [-2.620 2.530]; [-1.780 1.780]; [-2.890 2.890]];



end
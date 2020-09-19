function[chassis_config, joint_angle, wheel_rot] = NextState(vehicle_prop, direction, joints, wheel,...
    s_t_lim, const)
% Vehicle prop - [chassis orientation, vehicle_parameters, wheel location]
% direction - [free sliding angle, driving direction]
% joint = [joint angles, joint_locations, joint speed]
% wheel = [wheel speed, wheel location]
% s_t_lim = [max wheel speed, max joint speed, "dt-time step",...
%    "simulation duration (s)
% const = [wheel radius, number of wheels] 

% Calculation begins
% Ensure the velocities are within the boundary
wheel_speed = cell2mat(wheel(1));
joint_speed = cell2mat(joints(3));

% calculate the new points for angles and chasis
time_track = 0;
time_step = 1;

% wheel angle
wheel_rot= cell2mat(wheel(3));
joint_angle = cell2mat(joints(1));

% Vehicle param and exclude height
chassis_prop = cell2mat(vehicle_prop(1));

% looping till index
while time_track <= s_t_lim(4)
    % loop through and cap to fit within range.
    wheel_speed(wheel_speed > s_t_lim(1)) = s_t_lim(1);
    wheel_speed(wheel_speed < -1*s_t_lim(1)) = -1*s_t_lim(1);
    
    % loop through and cap to fit within range.
    joint_speed(joint_speed > s_t_lim(2)) = s_t_lim(2);
    joint_speed(joint_speed < -1*s_t_lim(2)) = -1*s_t_lim(2);
    
    % change in wheel angle and joint angle
    wheel_change = wheel_speed*s_t_lim(3);
    joint_change = joint_speed*s_t_lim(3);
    
    % increment wheel angle and joint angle
    wheel_rot = wheel_rot + wheel_change;
    joint_angle = joint_angle + joint_change;
    
    [~, F_6, ~] = ForwardKinematics(chassis_prop, joints, direction,...
    const, wheel);
        
    % calculate chassis change
    delta_q = ChassisChange(F_6, wheel_change, chassis_prop(1));   
    
    % new location
    chassis_prop(1:3) = chassis_prop(1:3) + delta_q';
              
    % Testing
    %str = sprintf("At time step %d, theta= %d, x-loc= %d, y-loc= %d\n",...
    %time_step, chassis_prop(1), chassis_prop(2), chassis_prop(3));
    %fprintf(str);    
        
    % increment the time track
    time_track = time_track + s_t_lim(3);
    time_step = time_step + 1;
end

chassis_config = chassis_prop;
 


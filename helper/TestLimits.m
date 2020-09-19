function[controls]=TestLimits(J_e, max_, joints, const, V)
% number of wheels
num_w = cell2mat(const(2));

% wheel speed limit
w_speed_lim = cell2mat(max_(1));

% joint speed limit
j_speed_lim = cell2mat(max_(2));

% joint angle
joint_angle = cell2mat(joints(1));

% restriction for joint all joints
joint_restrict = cell2mat(max_(4));

% obtain the linear and angular speeds
controls = pinv(J_e, 1e-2)*V;

 
% infinite loop for finding appropriate J_e
%while (true)
   
% seperate wheel and joint speed
u_s = controls(1:num_w);
theta_dot = controls(num_w + 1:end);
 
% counters for wheel speed, joint speed, and angles check
w_counter = 0;
js_counter = 0;
    
% loop through wheel_speed and 4 joint speeds
for ind=1:num_w
    % check wheel speed - if within bounds increment counter
    if abs(u_s(ind)) > w_speed_lim
        % zero column out
        J_e(:, ind) = zeros([6, 1]);           
    else
        w_counter = w_counter + 1;
    end

    % check joint speed - if within bounds increment counter
    if abs(theta_dot(ind)) > j_speed_lim
        J_e(:, ind + num_w) = zeros([6, 1]); 
    else
        js_counter = js_counter + 1;
    end
        
    % when at end check last joint speed
    if ind == num_w
        if abs(theta_dot(end)) > j_speed_lim
            J_e(:, end) = zeros([6, 1]);
        else
            js_counter = js_counter + 1;
        end
     end
% End of speed check
end
         
% Joint counter
ja_ends = 0;
 
% location
end_loc = [num_w+1:1:size(J_e,2)];

% Try to keep within restrictions -> After a number of loops go with zero
% configuration zeroed
% theta 1 and theta 5
loop = 1;
while loop > 0
    for end_check=1:size(end_loc,2)
        j_1 = joint_restrict(end_check,1);
        j_2 = joint_restrict(end_check,2);

        % ensure within boundary
        check = joint_angle(end_check) >= j_1 &&...
                joint_angle(end_check) <= j_2;

        % if we are out of bounds mark column
        if ~check               
            J_e(:, end_loc(end_check)) = zeros([6, 1]);
        else                
            ja_ends = ja_ends + 1;
        end
    end

    % check 1
    speed_check = js_counter + w_counter == 9;
    end_check = ja_ends == 5;

    % if we need to recalculate controls
    if ~(speed_check && end_check) 
        % re-calculate controls a number of times
       controls = pinv(J_e, 1e-4)*V;
    else
        % the limits are respected
        break;
    end
% End of Joint check
loop = loop - 1;
end
end
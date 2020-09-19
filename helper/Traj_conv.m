function[T_store]= Traj_conv(Traj, t_count)
% gripper state
grip_state = 0;

% states where gripper is closed
grip_closed = [3, 4, 5, 6];

% change gripper state
if size(grip_closed(grip_closed==t_count))
    grip_state=1;
end

% point storage
T_store = zeros(size(Traj,2), 13);

% loop through Traj
for ind=1:size(Traj,2)
    T = cell2mat(Traj(ind));
    [R, p] = TransToRp(T);
    
    % put in storage
    T_store(ind,:) = [reshape(R',[],9),p',grip_state];
end


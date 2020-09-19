function[T_total] = TrajectoryGenerator(T, k, chasis_prop, joints,...
    const_traj, direction, const, wheel)
    
% Generates end-effector references
[T_s] = T_Generation(T, chasis_prop, joints, direction,...
    const, wheel);

% obtain weighted time for each segment
% take out opening and closing motion from total time alloted for
% simulation
total_rem = cell2mat(const_traj(4))-0.625*2;

% dt per segment
dt1 = Time_Delta(cell2mat(T_s(1)), cell2mat(T_s(2)), const_traj);
dt2 = Time_Delta(cell2mat(T_s(2)), cell2mat(T_s(3)), const_traj);
dt3 = Time_Delta(cell2mat(T_s(2)), cell2mat(T_s(4)), const_traj);
dt4 = Time_Delta(cell2mat(T_s(4)), cell2mat(T_s(5)), const_traj);

% weighted time
d_weight = ([dt1, dt2, dt3, dt4]/(dt1+ 2*dt2 + dt3 + 2*dt4))*total_rem;

t_interval =[d_weight(1), d_weight(2), 0.625, d_weight(2), d_weight(3),...
    d_weight(4), 0.625, d_weight(4)];

% calculate preallocated positions
N_mat = zeros(size(t_interval));

for alloc_ind = 1:size(t_interval,2)
   % calculate N
   N_mat(alloc_ind) = round((t_interval(alloc_ind)/cell2mat(const_traj(8)) + 1)*k);
end

% Motion configuration
%T_s [T_se initial, T_se_standoff, T_se_grab, T_se standoff final,
% T_se release]
T_configure = {cell2mat(T_s(1)), cell2mat(T_s(2)),...
    cell2mat(T_s(2)),cell2mat(T_s(3)),...
    cell2mat(T_s(3)), cell2mat(T_s(3)),...
    cell2mat(T_s(3)), cell2mat(T_s(2)),...
    cell2mat(T_s(2)), cell2mat(T_s(4)),...
    cell2mat(T_s(4)), cell2mat(T_s(5)),...
    cell2mat(T_s(5)), cell2mat(T_s(5)),...
    cell2mat(T_s(5)), cell2mat(T_s(4))};

% Generate the trajectory
g_count = 1;
t_count = 1;
T_saved = zeros(4);
startp = 1;
endp = 0;

%storage for Traj generation
T_total = zeros(sum(N_mat),13);

while g_count < size(T_configure,2)
    % calculate N
    N = N_mat(t_count);
    
    % find end point
    endp = endp + N_mat(t_count); 

    % random number generater -> if odd use screw
    check = mod(randi(1000),3) == 0;
    
    % if not at closing or opening
    if (t_count ~= 3) && (t_count ~= 7)
        % based on user defined motion
        if cell2mat(const_traj(5)) == 1
            %disp("Here 1")
            % if 1 screw motion
            Traj = ScrewTrajectory(cell2mat(T_configure(g_count)),...
                cell2mat(T_configure(g_count + 1)),t_interval(t_count),...
                N, cell2mat(const_traj(6)));
            
        elseif cell2mat(const_traj(5)) == 2
            %disp("Here 2")
            % if 2 cartesian motion
            Traj = CartesianTrajectory(cell2mat(T_configure(g_count)),...
                cell2mat(T_configure(g_count + 1)), t_interval(t_count),...
                N, cell2mat(const_traj(6)));
            
        elseif cell2mat(const_traj(5)) == 3 && check
            %disp("Here 3")
            % user selected mixed motions and odd
            Traj = ScrewTrajectory(cell2mat(T_configure(g_count)),...
                cell2mat(T_configure(g_count + 1)),t_interval(t_count),...
                N, cell2mat(const_traj(6)));
        else 
            %disp("Here 4")
            % if 2 cartesian motion
            Traj = CartesianTrajectory(cell2mat(T_configure(g_count)),...
                cell2mat(T_configure(g_count + 1)), t_interval(t_count),...
                N, cell2mat(const_traj(6)));
        
        end
        
        % store last position if close to opening or closing
        if (t_count+ 1 == 3) || (t_count+ 1 == 7)
            T_saved = cell2mat(Traj(end));
        end
        
    % if opening or closing... keep constant
    else
        Traj = cell(1, N);
        for i = 1: N
            Traj{i} = T_saved;
        end
    end
    
    % Take trajectory and convert to 13 points
    T_total(startp:endp,:) = Traj_conv(Traj, t_count);
    
    
    % increment counters for loop
    g_count = g_count + 2;
    t_count = t_count + 1;
    
    % increment counters for storage
    startp = endp + 1;
    
end

end
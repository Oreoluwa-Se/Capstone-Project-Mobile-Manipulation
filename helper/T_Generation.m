function[T_s]=T_Generation(T, chasis_prop, joints,direction,...
    const, wheel)
% T_sc_init,T_sc_fin,T_ce_grab, T_ce_standoff

% calculate T_se
[T_se,~, ~] = ForwardKinematics(chasis_prop, joints,direction,...
    const, wheel);

% calculate T_se_standoff initial
T_se_standoff_init = cell2mat(T(1))*cell2mat(T(4));

% calculate T_se grab from initial position
T_se_grab_init = cell2mat(T(1))*cell2mat(T(3));

% calculate T_se_standoff final
T_se_standoff_fin = cell2mat(T(2))*cell2mat(T(4));

% calculate T_se grab from final position
T_se_grab_fin = cell2mat(T(2))*cell2mat(T(3));

T_s = {T_se, T_se_standoff_init, T_se_grab_init, T_se_standoff_fin,...
    T_se_grab_fin};
    
end
function[dt]=Time_Delta(T_a, T_b, consts)
% uses maximum angular and joint speed to calculate delta
% consts -> max wheel speed, radius, max joint speed

% calculate T_ab
T_ab = TransInv(T_a)*T_b;
% from T, extract start and end points (px,y,z
p_a = T_a(end, 1:3);
p_b = T_b(end, 1:3);

% find distance between both
dist = sqrt((p_a(1)-p_b(1))^2 + (p_a(2) - p_b(2))^2 +(p_a(3) - p_b(3))^2);

% dt based on linear distance
dT_lin = dist/(cell2mat(consts(1))*cell2mat(consts(2)));

% from t_ab extract R
[R,~] = TransToRp(T_ab);

% calculate the trace
tr = trace(R);

% criteria
if (R == eye(3))
    dt = dT_lin;
    
elseif tr == -1
    dt = max(dT_lin, pi/cell2mat(consts(3)));
    
else
    dt = max(dT_lin, acos(0.5*(tr - 1))/cell2mat(consts(3)));
end
    
end
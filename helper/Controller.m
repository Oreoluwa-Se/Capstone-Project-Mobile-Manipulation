function[controls, X_err] = Controller(chassis_prop, joints, T_s, gains,...
    direction, const, wheel, max_)
% dt
dt = cell2mat(max_(3));

% gains
Kp = cell2mat(gains(1)) * eye(6);
Ki = cell2mat(gains(2)) * eye(6);

% Xd, Xd,next
X_d = cell2mat(T_s(1));
X_dnext = cell2mat(T_s(2));

% current end-effector location
[T_se, ~, J_e] = ForwardKinematics(chassis_prop, joints, direction,...
    const, wheel);

% error and convert to velocity
X_err = se3ToVec(MatrixLog6(TransInv(T_se)*X_d));

% Vd -> reference twist that takes Xd to Xd,next
Vd = se3ToVec((1/dt)*MatrixLog6(TransInv(X_d)*X_dnext));

% caculating the twist
V = Adjoint(TransInv(T_se)*X_d)*Vd + Kp*X_err + Ki*X_err*dt;

% loop to check and recalculate
% Test limits
[controls] = TestLimits(J_e, max_, joints, const, V);

end

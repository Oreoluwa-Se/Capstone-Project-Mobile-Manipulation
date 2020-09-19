function[T_se, F_6, J_e] = ForwardKinematics(chassis_prop, joints, direction,...
    const, wheel)
% Wheel location
wheel_loc = cell2mat(wheel(2));

% Extract the joint angles and locations
j_angles = cell2mat(joints(1));
j_loc= cell2mat(joints(2));

% The T matrix for sb
T_sb = [cos(chassis_prop(1)), -sin(chassis_prop(1)), 0, chassis_prop(2);...
        sin(chassis_prop(1)), cos(chassis_prop(1)), 0, chassis_prop(3);...
        0, 0, 1, chassis_prop(4);...
        0, 0, 0, 1];

% The T matrix for end-effector to joint base
% M matrix
M_oe = [[eye(3);0, 0, 0],[j_loc(end,:), 1]'];

% The angle of rotation for each joint
omega_joints = [0,0,1; 0,-1,0; 0,-1,0; 0,-1,0; 0,0,1];

% calculating the screw axes
Screw_B = zeros(6, size(j_loc,1));

% body location
B_loc = j_loc(end,:);

% Generating the screw axis from the body
for ind = 1:size(j_loc,1)-1
    omega = omega_joints(ind,:);
    q = [j_loc(ind,1) - B_loc(1), j_loc(ind,2) - B_loc(2),...
        j_loc(ind,3) - B_loc(3)];
    v = cross(-omega, q);
    Screw_B(:,ind) = [omega,v];
end

% at end-effector
Screw_B(:,end) = [omega_joints(end,:), cross(-omega_joints(end,:),[0,0,0])];

% use FKinBody to fine the current T representation
T_oe = FKinBody(M_oe, Screw_B, j_angles');

% Body Jacobian
Jarm = JacobianBody(Screw_B, j_angles');

% Tbo
T_bo = [[eye(3);0 0 0], [0.1662, 0, 0.0026,1]'];

% calculate the current end-effector position
T_se = T_sb*T_bo*T_oe;

% H matrix calculation - free slid & driving directions, wheel radius,...
% heading angle
H_matrix = Hmatrix(direction, const, chassis_prop(1), wheel_loc);

% calculate F6 from F matrix
F_6 = [0 0 0 0; 0 0 0 0; pinv(H_matrix, 1e-4); 0 0 0 0];

% AdT -> T_eo*T_ob
ADT_eb = Adjoint(TransInv(T_oe)*TransInv(T_bo));

Jbase = ADT_eb*F_6;

J_e = [Jbase Jarm];



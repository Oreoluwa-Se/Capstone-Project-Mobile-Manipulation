function[delta_q]=ChassisChange(F6_matrix, wheel_delta, heading)

% the twist for the movement
req_twist = F6_matrix*wheel_delta';

% take mz, fx, fy from the chart
req_twist = req_twist(3:5);

% calculate delta q if twist is zero
if req_twist(1)==0
    delta_q = req_twist;

else
    % calculate delta q if twist is non-zero
    delta_q = [req_twist(1);...
        (req_twist(2)*sin(req_twist(1)) + req_twist(3)*(cos(req_twist(1))-1))/req_twist(1);...
        (req_twist(3)*sin(req_twist(1)) + req_twist(2)*(1-cos(req_twist(1))))/req_twist(1)];

end

% transofrm to chasis frame
delta_q = [1 0 0; 0 cos(heading) -sin(heading);...
            0 sin(heading) cos(heading)]*delta_q;
end
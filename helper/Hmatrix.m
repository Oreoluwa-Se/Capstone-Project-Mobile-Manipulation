function[H_matrix] = Hmatrix(direction, const, heading, loc_matrix)
% direction - [free sliding angle, driving direction]
% const = [wheel radius, number of wheels]
slide_matrix = cell2mat(direction(1));
drive_angle = cell2mat(direction(2));
const_ = cell2mat(const);

% h_matrix storage
H_matrix = zeros([size(drive_angle,1), 3]);

% calculate h matrix
for index=1:size(H_matrix, 1)
    % calculate the heading
    deg_conv = drive_angle(index);
            
    % calculate the sliding matrix
    slid_dir= slide_matrix(index);
    
    % angles
    theta1 = slid_dir + deg_conv;
    theta2 = theta1 + heading;
    mult = 1/(const_(1)*cos(slid_dir));

    % h_matrix
    H_matrix(index, :) = [loc_matrix(index,1)*sin(theta1) - loc_matrix(index,2)*cos(theta1);...
        cos(theta2); sin(theta2)]'*mult;
end

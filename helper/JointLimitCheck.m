function[count, J_e]= JointLimitCheck(J_e, j_angles, joint_restrict,...
        extreme_right, extreme_left, num_w)

j_angles = 0.01*j_angles;
j_angles(2)=-1.117;
count = 0;

% check joints 1 and 5
j1 = [j_angles(1),0,0,0, j_angles(end)];

for indx=1:size(j1,2)
    % check if the column is already zero then skip
    if isempty(find(J_e(:,indx + num_w) == 0, 1))
        continue;        
    end
    
    if j1(indx) ~= 0        
        % within the values increment counter
        if j1(indx) > joint_restrict(indx,1)...
                && j1(indx) < joint_restrict(indx,2)
            count = count + 1;
            continue;
        else
            % make zero
            J_e(:,indx + num_w) = zeros([6,1]);
        end
    end
end

% check joints 2,3,4 if theta 2 is zeroed
if round(abs(j_angles(2)),1) == 0
    j1 = [0,0,j_angles(3),j_angles(4),0];

    for indx=2:4
        % check if the column is already zero then skip
        if isempty(find(J_e(:,indx + num_w) == 0, 1))
            continue;        
        end

        if j1(indx) ~= 0        
            % within the values increment counter
            if j1(indx) > joint_restrict(indx,1)...
                    && j1(indx) < joint_restrict(indx,2)
                count = count + 1;
                continue;
            else
                % make zero
                J_e(:,indx + num_w) = zeros([6,1]);
            end
        end
    end

else % theta 2 not close enough to zero
    disp("here")

    % where theta 2 has moved to the right 
    if j_angles(2) < 0
        %dev
        disp(joint_restrict(2,1));
        dev = -((abs(joint_restrict(2,1)) + j_angles(2)));
        disp("dev")
        disp(dev)
        % calculate new maximum for theta 3 and 4
        max_34 =  extreme_right(2:3, 1) + [1.9*dev; -2*dev];
        disp("max_34")
        disp(max_34)
        % loop through both to check the new boundaries
        for n_check = 1:2
            if (max_34(n_check, 1) <= j_angles(3)) &&...
                    (max_34(n_check, 2) >= j_angles(3))
                count = count + 1;
            else
                J_e(:,(n_check + 2 + num_w)) = zeros([6,1]);                
            end
        end
        
    % where theta 2 has moved to the left    
    elseif j_angles(2) > 0
        % find how much we have deviated
        dev = abs(j_angles(2) - extreme_left(1,2));
        
        % calculate new maximum for theta 3 and 4
        max_34 =  extreme_left(2:3,:) - dev;
        
        % loop through both to check the new boundaries
        for n_check = 1:2
            if (max_34(n_check, 1) <= j_angles(3)) &&...
                    (max_34(n_check, 1) >= j_angles(3))
                count = count + 1;
            else
                J_e(:,(n_check + 2 + num_w)) = zeros([6,1]);                
            end
        end
    end
end

end
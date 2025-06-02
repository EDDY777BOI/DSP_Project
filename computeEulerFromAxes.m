function euler_deg = computeEulerFromAxes(R, a1, a2, a3, sequence)
% Compute Euler angles from a rotation matrix with a specified rotation sequence and custom axes.
%
%   euler_deg = computeEulerFromAxes(R, a1, a2, a3, sequence) extracts a 1x3 vector
%   of Euler angles (in degrees) from the 3x3 rotation matrix R, using custom rotation axes
%   provided by a1, a2, and a3. The string 'sequence' defines the intended intrinsic rotation
%   sequence. For example:
%
%       'YXY'  - First, rotate about a1 (to be aligned with [0;1;0]).
%                Then about a2, and finally about a3.
%
%       'ZXY'  - First, rotate about a1 (to be aligned with [0;0;1]).
%                Then about a2, and finally about a3.
%
%   INPUTS:
%       R        - 3x3 rotation matrix 
%       a1, a2, a3 - 3x1 vectors representing the first, second, and third rotation axes 
%       sequence  - A string indicating the rotation sequence to use like 'YXY'
%
%   OUTPUT:
%       euler_deg - 1x3 vector of Euler angles (in degrees). 


    %% --- Normalize the axes.
    tol = 1e-6;
    a1 = a1 / norm(a1);
    a2 = a2 / norm(a2);
    a3 = a3 / norm(a3);
    
    %% --- Switch between rotation sequences
    switch upper(sequence)
        case 'YXY'
            % --- For YXY: We want to force a1 to be the y-axis, i.e., [0; 1; 0].
            y_new = a1;
            % Project a2 onto the plane perpendicular to y_new to form the new x-axis.
            x_new = a2 - dot(a2, y_new) * y_new;
            if norm(x_new) < tol
                error('a2 is nearly parallel to a1 in YXY branch.');
            end
            x_new = x_new / norm(x_new);
            % Define the new z-axis to complete a right-handed system.
            z_new = cross(y_new, x_new);
            % Build the basis (change-of-basis) matrix Q.
            Q = [x_new, y_new, z_new];
            % Express R in the new coordinate system.
            R_new = Q' * R * Q;
            
            % --- Standard intrinsic Y–X–Y extraction formulas, 
            temp = max(min(R_new(2,2), 1), -1); % clamp the inputs to [-1, 1] so the acos can be calculated
            beta = acos(temp);
            if abs(sin(beta)) > tol
                gamma = atan2(R_new(1,2), R_new(3,2));
                alpha = atan2(R_new(2,1), -R_new(2,3));
            else
                % When beta is near 0 or pi, degrees of freedom are lost; use fallback.
                gamma = atan2(-R_new(3,1), R_new(1,1));
                alpha = 0;
            end
            euler_rad = [gamma, beta, alpha];
            
        case 'ZXY'
            % --- For ZXY: We want to force a1 to be the z-axis, i.e., [0; 0; 1].
            z_new = a1;
            % Project a2 onto the plane perpendicular to z_new to form the new x-axis.
            x_new = a2 - dot(a2, z_new) * z_new;
            if norm(x_new) < tol
                error('a2 is nearly parallel to a1 in ZXY branch.');
            end
            x_new = x_new / norm(x_new);
            % Define the new y-axis from the right-hand rule.
            y_new = cross(z_new, x_new);
            % Build the basis matrix Q.
            Q = [x_new, y_new, z_new];
            % Express the rotation matrix in the new coordinate system.
            R_new = Q' * R * Q;
            
            % --- Standard intrinsic Z–X–Y extraction.
            % One common set of formulas for an intrinsic ZXY rotation (i.e., R = Rz(gamma)*Rx(beta)*Ry(alpha))
            % is:
            %   beta = asin(-R_new(3,2))
            %   gamma = atan2(R_new(3,1), R_new(3,3))
            %   alpha = atan2(R_new(1,2), R_new(2,2))
            temp = max(min(-R_new(3,2), 1), -1); % Clamp the inputs to [-1, 1] so the asin can be calculated
            beta = asin(temp);
            if abs(cos(beta)) > tol
                gamma = atan2(R_new(3,1), R_new(3,3));
                alpha = atan2(R_new(1,2), R_new(2,2));
            else
                gamma = 0;
                alpha = atan2(-R_new(2,1), R_new(1,1));
            end
            euler_rad = [gamma, beta, alpha];
            
        otherwise
            error('Unsupported rotation sequence: %s', sequence);
    end

    %% --- unwrap and convert from radians to degrees
    euler_deg = rad2deg(euler_rad);
end

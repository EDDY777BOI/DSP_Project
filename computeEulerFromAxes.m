function euler_angles_deg = computeEulerFromAxes(R, axis1, axis2, axis3)
% Bereken Euler-hoeken uit rotatiematrix R op basis van opgegeven assenvolgorde
% IN:
%   R              - 3x3 rotatiematrix (bijv. bovenarm t.o.v. thorax)
%   axis1, axis2, axis3 - 3x1 vectoren die de rotatieassen voorstellen
%                          → volgorde bepaalt de rotatievolgorde
% UIT:
%   euler_angles_deg - 1x3 vector met Euler-hoeken in graden [angle1 angle2 angle3]

    % Normaliseer assen
    % Begin met normaliseren
    axis1 = axis1 / norm(axis1);
    axis2 = axis2 - dot(axis2, axis1) * axis1;
    axis2 = axis2 / norm(axis2);
    axis3 = cross(axis1, axis2);
    axis3 = axis3 / norm(axis3);
    axis2 = cross(axis3, axis1);  % Nu zijn alle assen orthonormaal (nodig voor rotm2eul)
    
    % Bepaal transformatie naar basis XYZ met jouw assen als kolommen
    T = [axis1, axis2, axis3];

    if det(T) < 0
        % Flip laatste as om het right-handed te maken
        T(:,3) = -T(:,3);
    end
    % DEBUG PRINT ortho moet ong. 0 zijn en det ong. 1
    %fprintf('Frame %d: Ortho = %.4f, det = %.2f\n', i, norm(T'*T - eye(3)), det(T));

    % Transformeer rotatiematrix naar deze nieuwe basis
    R_custom = T' * R * T;

    % Gebruik standaard 'ZYX'-volgorde maar op 'jouw' XYZ → dus euler1 rond axis1, enz.
    % De standaard MATLAB volgorde komt overeen met de inputvolgorde dankzij deze rotatie
    euler_angles_rad = rotm2eul(R_custom, 'ZYX');

    % De volgorde van rotaties is nu:
    %   rotatie 1 rond axis1 (jouw eerste input)
    %   rotatie 2 rond axis2
    %   rotatie 3 rond axis3
    % omdat T deze volgorde vastlegt

    % Omzetten naar graden
    euler_angles_deg = rad2deg(euler_angles_rad);

    % Herordenen zodat [angle1 angle2 angle3] overeenkomt met [axis1 axis2 axis3]
    euler_angles_deg = fliplr(euler_angles_deg);  % ZYX → jouw XYZ
end

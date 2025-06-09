function angles_unwrapped  = unwrapEulerAngles(angles_deg)
% Unwrap an Nx3 matrix of Euler-angles (in degrees)
    angles_rad = deg2rad(angles_deg);
    [N, nAngles] = size(angles_rad);
    angles_unwrapped_rad = zeros(N, nAngles);
    for i = 1:nAngles
        angles_unwrapped_rad(:, i) = unwrap(angles_rad(:, i));
    end
    % Convert back to degrees.
    angles_unwrapped = rad2deg(angles_unwrapped_rad);
end
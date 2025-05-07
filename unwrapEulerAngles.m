function angles_unwrapped_deg = unwrapEulerAngles(angles_deg)
% Unwrapt een Nx3 matrix met Euler-hoeken (in graden)
% Zet om naar radialen (dit moet voor de unwrap functie)
angles_rad = deg2rad(angles_deg);

% Pas unwrap toe 
angles_unwrapped_rad = unwrap(angles_rad);

% Zet terug naar graden
angles_unwrapped_deg = rad2deg(angles_unwrapped_rad);
end
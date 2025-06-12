function [angle_n,vel_n] = plotPhasePlane(angle, angVel, method)
    % first we normalize and then we plot a phase plane (angular veloctiy
    % vs angle)
    % first 2 parameters are of the same length (angle= angle in °, angVel = angular velocity in °/s)
    % the 3th parameter decides which normalization method will be used:
    %       - 'minmax'  : scale each to [–1, +1]
    %       - 'zscore'  : zero mean, unit std dev

    % we check to make sure the right type of data is passed:
    assert(isvector(angle) && isvector(angVel), 'First 2 parameters must be vectors');
    assert(numel(angle)==numel(angVel), 'Angle and angular velocity must match length');

    switch  lower(method)
        case 'minmax'
            angle_n = (angle - min(angle)) / (max(angle) - min(angle)) * 2 - 1;
            vel_n   = (angVel - min(angVel)) / (max(angVel) - min(angVel)) * 2 - 1;
        case 'zscore'
            angle_n = (angle - mean(angle)) / std(angle);
            vel_n   = (angVel - mean(angVel)) / std(angVel);
        otherwise
            error('Unknown method. Use "minmax" or "zscore".');
    end
      % Plot
    figure; hold on; grid on; axis equal;
    plot(angle_n, vel_n, '-', 'LineWidth', 1.5);
    xlabel('Normalized Angle');
    ylabel('Normalized Angular Velocity');
    title(sprintf('Phase Plane (%s normalization)', method));
    hold off

end


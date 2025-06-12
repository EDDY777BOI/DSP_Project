function plotEulerMotion(motion_case, angles)
    % Plot Euler angles for a given motion using switchcase
    % INPUTS:
    %   motion_case : 'shoulder', 'elbow', 'core', 'pelvis', 'thorax', 'knee'
    %   angles      : Nx3 matrix with Euler angles in degrees [gamma, beta, alpha]

    t = 0:(size(angles,1)-1);

    % Select title and axis labels based on the case
    switch lower(motion_case)
        case 'shoulder'
            titles = {'Plane of Elevation', 'Elevation', 'Axial Rotation'};
            mainTitle = 'Euler Angles – Shoulder Motion';

        case 'elbow'
            titles = {'Axial Rotation', 'Carrying Angle', 'Flexion/Extension'};
            mainTitle = 'Euler Angles – Elbow Motion';

        case 'core'
            titles = {'Lateral Flexion', 'Extension', 'Axial Rotation'};
            mainTitle = 'Euler Angles – Core Motion';

        case 'pelvis'
            titles = {'Pelvis X', 'Pelvis Y', 'Pelvis Z'};
            mainTitle = 'Euler Angles – Pelvis in Global Frame';

        case 'thorax'
            titles = {'Thorax X', 'Thorax Y', 'Thorax Z'};
            mainTitle = 'Euler Angles – Thorax in Global Frame';

        case 'knee'
            titles = {'Knee X', 'Knee Y', 'Knee Z'};
            mainTitle = 'Euler Angles – Left Knee Motion';

        otherwise
            error('Unknown motion type: %s', motion_case);
    end

    % Create the figure and subplots
    figure;
    sgtitle(mainTitle);
    subplot(3,1,1); plot(t, angles(:,1), 'LineWidth', 1.5); title(titles{1}); ylabel('deg'); grid on;
    subplot(3,1,2); plot(t, angles(:,2), 'LineWidth', 1.5); title(titles{2}); ylabel('deg'); grid on;
    subplot(3,1,3); plot(t, angles(:,3), 'LineWidth', 1.5); title(titles{3}); ylabel('deg'); xlabel('tijd (s)'); grid on;
end

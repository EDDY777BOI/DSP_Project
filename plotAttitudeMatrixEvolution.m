function plotAttitudeMatrixEvolution(R, segment_name)
    % Plot evolution of rotation matrix components over time
    % R is Nx3x3
    % segment_name is a string for plot titles

    N = size(R, 1);
    colors = ['r', 'g', 'b'];

    figure('Name', ['Attitude Matrix - ' segment_name]);
    for row = 1:3
        subplot(3,1,row);
        hold on;
        for col = 1:3
            plot(1:N, squeeze(R(:,row,col)), colors(col));
        end
        title([segment_name ' - Row ' num2str(row)]);
        ylabel('Value'); grid on;
        legend('X', 'Y', 'Z');
    end
    xlabel('Frame');
end

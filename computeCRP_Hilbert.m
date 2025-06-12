function crp_hilbert = computeCRP_Hilbert(angle1, angle2, t)
    % Compute CRP using the Hilbert Transform method
    % Inputs:
    %   angle1, angle2 - angle signals (e.g., in degrees or radians)
    %
    % Output:
    %   crp_hilbert - continuous relative phase in degrees, wrapped to [-180, +180]

    % 1. Convert to analytic signals
    analytic1 = hilbert(angle1);
    analytic2 = hilbert(angle2);

    % 2. Extract instantaneous phases
    phase1 = unwrap(angle(analytic1));  % radians
    phase2 = unwrap(angle(analytic2));  % radians

    phase1_deg = rad2deg(phase1);  % convert to degrees
    phase2_deg = rad2deg(phase2);


    % 3. CRP = difference in phase angles
    % Convert to degrees and wrap to [-180, 180]
    crp_hilbert = wrapTo180(phase1_deg - phase2_deg);


    figure;
    %subplot(2,1,1);
    plot(t, phase1_deg, 'b', t, phase2_deg, 'r', 'LineWidth', 1.5);
    title('Hilbert Phase Angles');
    legend('Segment 1', 'Segment 2');
    ylabel('Phase (°)'); grid on;

    % subplot(2,1,2);
    % plot(t, crp_hilbert, 'k', 'LineWidth', 1.5);
    % title('CRP (Hilbert Method)');
    % xlabel('Time (s)');
    % ylabel('CRP (°)'); grid on;
end

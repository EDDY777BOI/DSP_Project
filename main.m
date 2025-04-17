%BRANCH SAM
%% Import and visualize TSV file
clc; clear; close all;

% Import of the TSV file
filename = '10Ax1.tsv';
data = readtable(filename, 'FileType', 'text', 'Delimiter', '\t');
fs = 300;           % samplefrequentie in Hz
dt = 1/fs;          % tijdsstap
t = (0:height(data)-1)' / fs;  % tijdsvector

% Uncomment the line below to show first 5 rows as control
% disp(data(1:5, :));

% Check for missing values
% missing_data = ismissing(data);

% Print how much NaNs there are per column
% num_missing = sum(missing_data);
% disp('Aantal ontbrekende waarden per kolom:');
% disp(num_missing);

% Filterparameters (Butterworth)
fc = 10;  % cutoff frequentie
[b, a] = butter(2, fc / (fs/2));

% Filter all market coordinates (X, Y, Z) individually
filtered_data = data;  % new table with filtered data
for i = 1:width(data)
    column = data{:, i};
    filtered_data{:, i} = filtfilt(b, a, column);
end

% Finding marker names
all_vars = data.Properties.VariableNames;             % get all the names of the columns
markers = unique(regexprep(all_vars, '[XYZ]$', ''));  % unique marker labels


%% Lokale assenstelsel UPPER ARM RIGHT (U) – volgens ISB

amount_frames = height(filtered_data);
U = zeros(amount_frames, 3, 3);  % 3x3 matrix per frame, we maken dus 558 3x3 matrixen

% Coördinaten ophalen
AR = [filtered_data.ARX, filtered_data.ARY, filtered_data.ARZ];
ELR = [filtered_data.ELRX, filtered_data.ELRY, filtered_data.ELRZ];
EMR = [filtered_data.EMRX, filtered_data.EMRY, filtered_data.EMRZ];
PLR = [filtered_data.PLRX, filtered_data.PLRY, filtered_data.PLRZ];

% DE KERN VAN HOE EEN LOKAAL ASSTENSTELSEL/ROTATIEMATRIX WORDT GEBOUWD
% Doorloop eerst alle frames
for i = 1:amount_frames
    % Humerus-epicondyl-punten (elleboog): lateraal & mediaal
    elbow_lat = ELR(i, :);
    elbow_med = EMR(i, :);
    
    % Z-as: van schouder naar midden elleboog
    midpoint_elbow = 0.5 * (elbow_lat + elbow_med);
    Z = normalize(PLR(i, :) - midpoint_elbow);  % van elleboog naar pols

    % X-as: dwarsas (van laterale naar mediale epicondyl)
    X = normalize(elbow_lat - elbow_med);  % van med naar lat

    % Y-as: orthogonaal (kruisproduct) = voorwaartse rotatieas
    Y = cross(Z, X);

    % Her-orthogonaliseren voor zekerheid (optioneel)
    X = cross(Y, Z);  % herbereken X zodat alle 3 orthogonaal

    % Attitude matrix (kolommen zijn assen)
    U(i, :, :) = [X; Y; Z]';
end

disp('Attitude matrix U (Upper Arm Right) aangemaakt.');


%% Visualisatie van lokale assenstelsel Upper Arm Right (U)

figure;
hold on;
axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on;
title('Lokale assenstelsels Upper Arm Right (U) over tijd');
view(3);

% Aantal frames om te tonen (om de 50 bv.)
step = 10;

for i = 1:step:height(filtered_data)
    origin = AR(i, :);  % schouderpunt

    % Extract attitude matrix U op frame i
    R = squeeze(U(i, :, :));

    % Assen (kolommen)
    X = R(:, 1);
    Y = R(:, 2);
    Z = R(:, 3);

    scale = 100;  % lengte van de assen

    % Plot assen met quiver3
    quiver3(origin(1), origin(2), origin(3), scale*X(1), scale*X(2), scale*X(3), 'r', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Y(1), scale*Y(2), scale*Y(3), 'g', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Z(1), scale*Z(2), scale*Z(3), 'b', 'LineWidth', 1.5);
end
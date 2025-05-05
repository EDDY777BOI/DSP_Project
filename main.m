%% Import and visualize TSV file
%clc; clear; close all;

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

%% Lokale assenstelsel RIGHT Forearm (F) – volgens ISB
% Hebben de Y as van F nodig voor U, dus definieren eerst deze

amount_frames = height(filtered_data);
F = zeros(amount_frames, 3, 3);  % 3x3 matrix per frame, we maken dus 558 3x3 matrixen

% Coördinaten ophalen
PLR = [filtered_data.PLRX, filtered_data.PLRY, filtered_data.PLRZ];
PMR = [filtered_data.PMRX, filtered_data.PMRY, filtered_data.PMRZ];
ELR = [filtered_data.ELRX, filtered_data.ELRY, filtered_data.ELRZ];
EMR = [filtered_data.EMRX, filtered_data.EMRY, filtered_data.EMRZ]; 


% Doorloop eerst alle frames
for i = 1:amount_frames

    % Epicondylus lateralis/medialis right punten (elleboog)
    elbow_lat = ELR(i, :);
    elbow_med = EMR(i, :);

    % styloid lateralis/medialis right punten (pols)
    wrist_lat= PLR(i,:);
    wrist_med = PMR(i,:);

    % Y-as: van PMR naar midden elleboog
    midpoint_elbow = 0.5 * (elbow_lat + elbow_med);
    Y = normalize(midpoint_elbow - PLR(i, :));  % van pols (ISB:PMR, maar project description zegt gebruik PLR) naar elleboog

    % X-as: loodrecht op vlak gevormd door PMR, PLR, midpoint_elbow
    v1 = PLR(i,:) - wrist_med;
    v2 = midpoint_elbow - wrist_med;
    X = normalize(cross(v1,v2));  % kruisproduct van de 2 vlakken

    % Z-as: orthogonaal (kruisproduct) = voorwaartse rotatieas
    Z = cross(X, Y);

    % Her-orthogonaliseren voor zekerheid (optioneel)
    %X = cross(Y, Z);  % herbereken X zodat alle 3 orthogonaal zijn

    % Van de vectoren eenheidsvectoren maken
    X = Unity(X);
    Z = Unity(Z);
    Y = Unity(Y);
    % Attitude matrix (kolommen zijn assen)
    F(i, :, :) = [X; Y; Z]';
end

disp('Attitude matrix F (Forearm Right) aangemaakt.');

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
    midpoint_elbow = 0.5 * (elbow_lat + elbow_med);
    
    % Deze assen waren volgens mij niet juist gedefinieerd

    % % Z-as: van schouder naar midden elleboog
    % midpoint_elbow = 0.5 * (elbow_lat + elbow_med);
    % Z = normalize(PLR(i, :) - midpoint_elbow);  % van elleboog naar pols
    % % X-as: dwarsas (van laterale naar mediale epicondyl)
    % X = normalize(elbow_lat - elbow_med);  % van med naar lat
    % % Y-as: orthogonaal (kruisproduct) = voorwaartse rotatieas
    % Y = cross(Z, X);
    % % Her-orthogonaliseren voor zekerheid (optioneel)
    % X = cross(Y, Z);  % herbereken X zodat alle 3 orthogonaal zijn

    % Y-as: van midden elleboog naar schouder, richting schouder (AR)
    Y = normalize(AR(i,:) - midpoint_elbow);  

    % Z-as: lijn loodrecht op vlak gemaakt door Y-as en Y-as van Forearm,
    % naar rechts gericht
    Yf = squeeze(F(i, :, 2));  % Y-as van forearm
    %Yf = normalize(midpoint_elbow - PLR(i, :)); 

    Z = cross(Yf,Y);
    
    % X-as: lijn loodrecht op Y en Z, gericht naar voor
    X = cross(Y,Z);

    % Van de vectoren eenheidsvectoren maken
    X = Unity(X);
    Z = Unity(Z);
    Y = Unity(Y);

    % Attitude matrix (kolommen zijn assen)
    U(i, :, :) = [X; Y; Z]';
end

disp('Attitude matrix U (Upper Arm Right) aangemaakt.');


%% Lokale assenstelsel Thorax (T) – volgens ISB

amount_frames = height(filtered_data);
T = zeros(amount_frames, 3, 3);  % 3x3 matrix per frame, we maken dus 558 3x3 matrixen

% Coördinaten ophalen
PX = [filtered_data.PXX, filtered_data.PXY, filtered_data.PXZ];
T7 = [filtered_data.T7X, filtered_data.T7Y, filtered_data.T7Z];
MS = [filtered_data.MSX, filtered_data.MSY, filtered_data.MSZ];
C7 = [filtered_data.C7X, filtered_data.C7Y, filtered_data.C7Z]; 


% Doorloop eerst alle frames
for i = 1:amount_frames
    
    midpoint_lower_T = 0.5 * (PX(i,:) + T7(i,:));
    midpoint_higher_T = 0.5 * (MS(i,:) + C7(i,:));

    % Y-as: van  midpoint_MS_C7 naar midpoint_PX_T8 naar boven gericht
    Y = normalize(midpoint_higher_T - midpoint_lower_T); 

    % Z-as: loodrecht op vlak gevormd door MS, C7 en midpoint_lower_T naar
    % rechts gericht
    v1 = C7(i,:) - MS(i,:);
    v2 = midpoint_lower_T - MS(i,:);
    Z = normalize(cross(v1,v2));  % kruisproduct van de 2 vlakken
    
    % X-as: orthogonaal (kruisproduct) = voorwaartse rotatieas
    X = cross(Y, Z);

    % Her-orthogonaliseren voor zekerheid (optioneel)
    Y = cross(Z, X);  % herbereken X zodat alle 3 orthogonaal zijn

    % Van de vectoren eenheidsvectoren maken
    X = Unity(X);
    Z = Unity(Z);
    Y = Unity(Y);

    % Attitude matrix (kolommen zijn assen)
    T(i, :, :) = [X; Y; Z]';
end

disp('Attitude matrix T (Thorax) aangemaakt.');

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

    % % Plot assen met quiver3
    % quiver3(origin(1), origin(2), origin(3), scale*X(1), scale*X(2), scale*X(3), 'r', 'LineWidth', 1.5);
    % quiver3(origin(1), origin(2), origin(3), scale*Y(1), scale*Y(2), scale*Y(3), 'g', 'LineWidth', 1.5);
    % quiver3(origin(1), origin(2), origin(3), scale*Z(1), scale*Z(2), scale*Z(3), 'b', 'LineWidth', 1.5);

    if i == 1
        qx = quiver3(origin(1), origin(2), origin(3), scale*X(1), scale*X(2), scale*X(3), 'r', 'LineWidth', 1.5);
        qy = quiver3(origin(1), origin(2), origin(3), scale*Y(1), scale*Y(2), scale*Y(3), 'g', 'LineWidth', 1.5);
        qz = quiver3(origin(1), origin(2), origin(3), scale*Z(1), scale*Z(2), scale*Z(3), 'b', 'LineWidth', 1.5);
    else
        quiver3(origin(1), origin(2), origin(3), scale*X(1), scale*X(2), scale*X(3), 'r', 'LineWidth', 1.5);
        quiver3(origin(1), origin(2), origin(3), scale*Y(1), scale*Y(2), scale*Y(3), 'g', 'LineWidth', 1.5);
        quiver3(origin(1), origin(2), origin(3), scale*Z(1), scale*Z(2), scale*Z(3), 'b', 'LineWidth', 1.5);
    end
end

legend([qx qy qz], {'X-as (voorwaarts)', 'Y-as (proximaal)', 'Z-as (rechts)'});

%% %% Visualisatie van lokale assenstelsel Forearm Right (F)

figure;
hold on;
axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on;
title('Lokale assenstelsels Forearm Right (F) over tijd');
view(3);

% Om de 10 frames visualizeren we
step = 10;
scale = 100;  % lengte van de assen

for i = 1:step:height(filtered_data)
    origin = PMR(i, :);  % Volgens ISB: US ~ PMR

    % Extract attitude matrix F op frame i
    R = squeeze(F(i, :, :));

    % Assen (kolommen)
    X = R(:, 1);
    Y = R(:, 2);
    Z = R(:, 3);

    % Plot assen met quiver3
    quiver3(origin(1), origin(2), origin(3), scale*X(1), scale*X(2), scale*X(3), 'r', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Y(1), scale*Y(2), scale*Y(3), 'g', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Z(1), scale*Z(2), scale*Z(3), 'b', 'LineWidth', 1.5);
end

%% %% Visualisatie van lokale assenstelsel Thorax (T)

figure;
hold on;
axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on;
title('Lokale assenstelsels Thorax (T) over tijd');
view(3);

% Om de 10 frames visualizeren we
step = 10;
scale = 100;  % lengte van de assen

for i = 1:step:height(filtered_data)
    origin = MS(i, :);  % Volgens ISB: IJ ~ MS

    % Extract attitude matrix T op frame i
    R = squeeze(T(i, :, :));

    % Assen (kolommen)
    X = R(:, 1);
    Y = R(:, 2);
    Z = R(:, 3);

    % Plot assen met quiver3
    quiver3(origin(1), origin(2), origin(3), scale*X(1), scale*X(2), scale*X(3), 'r', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Y(1), scale*Y(2), scale*Y(3), 'g', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Z(1), scale*Z(2), scale*Z(3), 'b', 'LineWidth', 1.5);
end


%% Visualisatie van lokale assenstelsels Left Thigh (TL)

figure;
hold on;
axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on;
title('Lokale assenstelsels Left Thigh (TL) over tijd');
view(3);

% Om de 10 frames visualiseren
step = 10;
scale = 100;  % lengte van de assen

for i = 1:step:height(filtered_data)
    origin = HL(i, :);  % Origin = heup (zoals ingesteld in de TL-matrix)

    % Extract attitude matrix TL op frame i
    R = squeeze(TL(i, :, :));

    % Assen (kolommen)
    X = R(:, 1);
    Y = R(:, 2);
    Z = R(:, 3);

    % Plot assen met quiver3
    quiver3(origin(1), origin(2), origin(3), scale*X(1), scale*X(2), scale*X(3), 'r', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Y(1), scale*Y(2), scale*Y(3), 'g', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Z(1), scale*Z(2), scale*Z(3), 'b', 'LineWidth', 1.5);
end

%% Gecombineerde visualisatie van de segmenten

figure;
hold on;
axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on;
title('Gecombineerde lokale assenstelsels');
view(3);

scale = 100;   % lengte van de assen
step = 10;

for i = 1:step:amount_frames
    % Upper Arm (U)
    origin_U = AR(i, :);                  % schouder als oorsprong
    RU = squeeze(U(i, :, :));             % attitude matrix U
    XU = RU(:,1); YU = RU(:,2); ZU = RU(:,3);
    
    quiver3(origin_U(1), origin_U(2), origin_U(3), scale*XU(1), scale*XU(2), scale*XU(3), 'r', 'LineWidth', 1.5);
    quiver3(origin_U(1), origin_U(2), origin_U(3), scale*YU(1), scale*YU(2), scale*YU(3), 'g', 'LineWidth', 1.5);
    quiver3(origin_U(1), origin_U(2), origin_U(3), scale*ZU(1), scale*ZU(2), scale*ZU(3), 'b', 'LineWidth', 1.5);

    % Forearm (F)
    origin_F = PMR(i, :);                 % pols (PMR) als oorsprong (~ US)
    RF = squeeze(F(i, :, :));             % attitude matrix F
    XF = RF(:,1); YF = RF(:,2); ZF = RF(:,3);
    
   
    quiver3(origin_F(1), origin_F(2), origin_F(3), scale*XF(1), scale*XF(2), scale*XF(3), 'm', 'LineWidth', 1.5);  
    quiver3(origin_F(1), origin_F(2), origin_F(3), scale*YF(1), scale*YF(2), scale*YF(3), 'c', 'LineWidth', 1.5);  
    quiver3(origin_F(1), origin_F(2), origin_F(3), scale*ZF(1), scale*ZF(2), scale*ZF(3), 'k', 'LineWidth', 1.5);  

    % Thorax (T)
    origin_T = MS(i,:);                   % Ms als oorsprong (~IJ)
    RT = squeeze(T(i,:,:));               % attitude matrix T
    XT = RT(:,1); YT = RT(:,2); ZT = RT(:,3);

    quiver3(origin_T(1), origin_T(2), origin_T(3), scale*XT(1), scale*XT(2), scale*XT(3), 'k', 'LineWidth', 1.5);    
    quiver3(origin_T(1), origin_T(2), origin_T(3), scale*YT(1), scale*YT(2), scale*YT(3), 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5);
    quiver3(origin_T(1), origin_T(2), origin_T(3), scale*ZT(1), scale*ZT(2), scale*ZT(3), 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); % grijze kleur

end
legend('Ux', 'Uy', 'Uz', 'Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz');


%% Rotatie matrices & Euler/Cardan angles

amount_frames = height(filtered_data);
R_UT = zeros(amount_frames,3,3); 
for i = 1:amount_frames
    RU = squeeze(U(i,:,:)); % 3x3 upper arm
    RT = squeeze(T(i,:,:)); % 3x3 thorax
    RF = squeeze(F(i,:,:)); % 3x3 forearm
    
    % Relatieve matrix: upper arm in thorax‑coördinaten
    R_rel_UT = RT.' * RU; 
    R_UT(i,:,:) = R_rel_UT;

    % Relatieve matrix: forarm in upper arm coördinaten
    R_rel_FU = RU.' * RF;

   
    % Euler-hoeken voor Shoulder motion based on R_rel_UT (ISB: Y-X-Y volgorde)
    euler_rad = rotm2eul(R_rel_UT, 'YXY');  % [gamma beta alpha] in radialen
    euler_shoulder_rad(i,:) = euler_rad;
end
    % Check of de rotatiematrices orthonormaal zijn, dit is vereist voor de
    % rotm2eul functie
    orthonormaal = norm(R_rel_UT * R_rel_UT.' - eye(3)) < 1e-6; % moet true zijn
    determinant = det(R_rel_UT); % moet dicht bij 1 liggen
    disp(orthonormaal);
    disp(determinant);

disp('R_rel_UT (Upper arm relative to Thorax) aangemaakt.');
disp('R_rel_FU (Forearm relative to Upper arm) aangemaakt.');


%% EULER/CARDAN angles in degrees

% Omzetten naar graden
euler_shoulder_deg = rad2deg(euler_shoulder_rad);

% Optioneel opsplitsen
gamma  = euler_shoulder_deg(:,1);  % plane of elevation
beta   = euler_shoulder_deg(:,2);  % elevation
alpha  = euler_shoulder_deg(:,3);  % axial rotation

% Struct voor overzicht
ShoulderAngles = table(gamma, beta, alpha,'VariableNames', {'PlaneOfElevation_deg','Elevation_deg','AxialRotation_deg'});

% Toon eerste paar waarden
disp('Eerste 10 rijen van de schouderhoeken (Euler/Cardan):');
disp(ShoulderAngles(1:100,:));

%% Lokale assenstelsel LEFT THIGH (TL) – origin = HL, Z-as naar lateraal

amount_frames = height(filtered_data);
TL = zeros(amount_frames, 3, 3);  % 3x3 matrix per frame

% Coördinaten ophalen
HL = [filtered_data.HLX, filtered_data.HLY, filtered_data.HLZ];         % heup links
CLL = [filtered_data.CLLX, filtered_data.CLLY, filtered_data.CLLZ];     % condylus lateralis links
CML = [filtered_data.CMLX, filtered_data.CMLY, filtered_data.CMLZ];     % condylus medialis links

for i = 1:amount_frames
    % Kniepunten
    knee_lat = CLL(i, :);
    knee_med = CML(i, :);
    knee_mid = 0.5 * (knee_lat + knee_med);

    % Origin verplaatst naar HL
    origin = HL(i, :);

    % X-as: van mediale naar laterale condylus (dwarsas) → richting buitenzijde
    X = normalize(knee_lat - knee_med);  % lateraal gericht

    % Z-as: orthogonaal op vlak gevormd door HL en kniecondylen, richting buiten
    v1 = knee_lat - origin;
    v2 = knee_med - origin;
    Z = normalize(cross(v1, v2));  % lateraal gericht loodrecht vlak

    % Y-as: orthogonaal op Z en X, naar voor
    Y = cross(Z, X);

    % Her-orthogonaliseren
    X = cross(Y, Z);

    % Eenheidsvectoren
    X = Unity(X);
    Y = Unity(Y);
    Z = Unity(Z);

    % Attitude matrix (kolommen = assen, origin = HL)
    TL(i, :, :) = [X; Y; Z]';
end

disp('Attitude matrix TL (Left Thigh) aangemaakt.');


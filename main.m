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
    %Yf = normalize(midpoint_elbow - PLR(i, :)); % y as Forearm 
    RF = squeeze(F(i, :, :));
    Yf = RF(:,2); % 2e kolom de Y as
    Z = cross(Y,Yf);
    
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

%% Lokale assenstelsel Pelvic (P) – volgens ISB

amount_frames = height(filtered_data);
P = zeros(amount_frames, 3, 3);  % 3x3 matrix per frame, we maken dus 558 3x3 matrixen

% Coördinaten ophalen
SIASL = [filtered_data.SIASLX, filtered_data.SIASLY, filtered_data.SIASLZ];
SIASR = [filtered_data.SIASRX, filtered_data.SIASRY, filtered_data.SIASRZ];
SIPSL = [filtered_data.SIPSLX, filtered_data.SIPSLY, filtered_data.SIPSLZ];
SIPSR = [filtered_data.SIPSRX, filtered_data.SIPSRY, filtered_data.SIPSRZ]; 
HL    = [filtered_data.HLX, filtered_data.HLY, filtered_data.HLZ];

% Doorloop eerst alle frames
for i = 1:amount_frames
    origin = HL(i, :);  % Hip Left
    midpoint_SIPS = 0.5 * (SIPSR(i,:) + SIPSL(i,:));

    % Z-as: tussen SIASR en SIASR, naar links gericht
    Z = normalize(SIASL(i,:) - SIASR(i,:)); 

    % X-as: orthogonaal op Z en parallel met lijn in het vlak gevormd door SIASR en SIASL en
    % midpoint_SIPS
    plane = cross(midpoint_SIPS - SIASL(i,:), SIASR(i,:) - SIASL(i,:));
    X = cross(Z, plane);

    % Y-as: loodracht op X en Z, naar boven gericht
    Y = cross(X, Z); 
    % Her-orthogonaliseren voor zekerheid (optioneel)
   
    % Van de vectoren eenheidsvectoren maken
    X = Unity(X);
    Z = Unity(Z);
    Y = Unity(Y);

    % Attitude matrix (kolommen zijn assen)
    P(i, :, :) = [X; Y; Z]';
end

disp('Attitude matrix P (Pelvic) aangemaakt.');

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

    % Y-as: lijn tussen midpunt CLL/CML en oorsprong, naar boven gericht
    Y = normalize(origin - knee_mid);

    % Z-as: loodrecht op de y as, in het vlak gevormd door de oorsprong en
    % CML/CLL, naar links gericht
    v1 = CML(i,:) - origin;
    v2 = CLL(i,:) - origin;
    Z_temp = cross(v1, v2); % kruisproduct van de 2 vectoren, naar voor gericht
    Z = cross(Z_temp, Y);

    % X-as: kruisproduct Y en Z, naar voor gericht (anterior)
    X = cross(Y,Z);

    % Her-orthogonaliseren
    Z = cross(Y, X);

    % % X-as: van mediale naar laterale condylus (dwarsas) → richting buitenzijde
    % X = normalize(knee_lat - knee_med);  % lateraal gericht
    % 
    % % Z-as: orthogonaal op vlak gevormd door HL en kniecondylen, richting buiten
    % v1 = knee_lat - origin;
    % v2 = knee_med - origin;
    % Z = normalize(cross(v1, v2));  % lateraal gericht loodrecht vlak
    % 
    % % Y-as: orthogonaal op Z en X, naar voor
    % Y = cross(Z, X);
    % 
    % % Her-orthogonaliseren
    % X = cross(Y, Z);

    % Eenheidsvectoren
    X = Unity(X);
    Y = Unity(Y);
    Z = Unity(Z);

    % Attitude matrix (kolommen = assen, origin = HL)
    TL(i, :, :) = [X; Y; Z]';
end

disp('Attitude matrix TL (Left Thigh) aangemaakt.');

%% Lokale assenstelsel LEFT SHANK (SL) -- onderbeen

amount_frames = height(filtered_data);
SL = zeros(amount_frames, 3, 3);  % 3x3 matrix per frame

% Coördinaten ophalen
MLL = [filtered_data.MLLX, filtered_data.MLLY, filtered_data.MLLZ];     % malleolus lateralis links
MML = [filtered_data.MMLX, filtered_data.MMLY, filtered_data.MMLZ];     % malleolus medialis links
CLL = [filtered_data.CLLX, filtered_data.CLLY, filtered_data.CLLZ];     % condylus lateralis links
CML = [filtered_data.CMLX, filtered_data.CMLY, filtered_data.CMLZ];     % condylus medialis links

for i = 1:amount_frames
    % oorsprong tussen MLL en MML
    origin = 0.5 * (MLL(i,:)) + (MML(i,:));

    % Z-as: van mediale naar laterale malleolus naar links gericht
    Z = normalize(MLL(i,:) - MML(i,:));
 
    % X-as: loodrecht op torsionaal vlak van onderbeen (gevormd door
    % CML/CLL en MML/MLL
    v1 = CLL(i,:) - MML(i,:);
    v2 = CML(i,:) - MLL(i,:);
    X = cross(v1,v2);

    % Y-as: orthogonaal op Z en X, naar boven
    Y = cross(X, Z);

    % Her-orthogonaliseren
    X = cross(Y, Z);

    % Eenheidsvectoren
    X = Unity(X);
    Y = Unity(Y);
    Z = Unity(Z);

    % Attitude matrix (kolommen = assen, origin = HL)
    SL(i, :, :) = [X; Y; Z]';
end

disp('Attitude matrix SL (Left Shank) aangemaakt.');

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

%% %% Visualisatie van lokale assenstelsel Pelvic (P)

figure;
hold on;
axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on;
title('Lokale assenstelsels Pelvic (P) over tijd');
view(3);

% Om de 10 frames visualizeren we
step = 10;
scale = 100;  % lengte van de assen

for i = 1:step:height(filtered_data)
    origin = HL(i, :);  % Hip Left

    % Extract attitude matrix T op frame i
    R = squeeze(P(i, :, :));

    % Assen (kolommen)
    X = R(:, 1);
    Y = R(:, 2);
    Z = R(:, 3);

    % Plot assen met quiver3
    quiver3(origin(1), origin(2), origin(3), scale*X(1), scale*X(2), scale*X(3), 'r', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Y(1), scale*Y(2), scale*Y(3), 'g', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Z(1), scale*Z(2), scale*Z(3), 'b', 'LineWidth', 1.5);
end
legend('X-as (voor)', 'Y-as (boven)', 'Z-as (links)');

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
legend('X-as (voor)', 'Y-as (boven)', 'Z-as (links)');


%% Visualisatie van lokale assenstelsels Left Shanks (SL)

figure;
hold on;
axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on;
title('Lokale assenstelsels Left Shank (SL) over tijd');
view(3);

% Om de 10 frames visualiseren
step = 10;
scale = 100;  % lengte van de assen

for i = 1:step:height(filtered_data)
    % oorsprong tussen MLL en MML
    origin = 0.5 * (MLL(i,:)) + (MML(i,:));

    % Extract attitude matrix TL op frame i
    R = squeeze(SL(i, :, :));

    % Assen (kolommen)
    X = R(:, 1);
    Y = R(:, 2);
    Z = R(:, 3);

    % Plot assen met quiver3
    quiver3(origin(1), origin(2), origin(3), scale*X(1), scale*X(2), scale*X(3), 'r', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Y(1), scale*Y(2), scale*Y(3), 'g', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*Z(1), scale*Z(2), scale*Z(3), 'b', 'LineWidth', 1.5);
end
    legend('X-as (voor)', 'Y-as (boven)', 'Z-as (links)');

%% Gecombineerde visualisatie van Upper arm (U) en Thorax (T)

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

    % Thorax (T)
    origin_T = MS(i,:);                   % Ms als oorsprong (~IJ)
    RT = squeeze(T(i,:,:));               % attitude matrix T
    XT = RT(:,1); YT = RT(:,2); ZT = RT(:,3);

    quiver3(origin_T(1), origin_T(2), origin_T(3), scale*XT(1), scale*XT(2), scale*XT(3), 'k', 'LineWidth', 1.5);    
    quiver3(origin_T(1), origin_T(2), origin_T(3), scale*YT(1), scale*YT(2), scale*YT(3), 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5);
    quiver3(origin_T(1), origin_T(2), origin_T(3), scale*ZT(1), scale*ZT(2), scale*ZT(3), 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5); % grijze kleur

end
legend('Ux', 'Uy', 'Uz', 'Tx', 'Ty', 'Tz');


%% Gecombineerde visualisatie van Pelvis (P) en Left Thigh (TL)

figure;
hold on;
axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on;
title('Pelvis (P) & Left Thigh (TL) – Gecombineerde visualisatie');
view(3);

scale = 100;   % lengte van de assen
step = 10;

for i = 1:step:amount_frames
    % Origin = HL (heup links)
    origin = HL(i, :);

    % Extract attitude matrices
    RP = squeeze(P(i, :, :));  % Pelvis
    RTL = squeeze(TL(i, :, :)); % Left Thigh

    % Pelvis-assen
    XP = RP(:, 1); YP = RP(:, 2); ZP = RP(:, 3);
    % Thigh-assen
    XTL = RTL(:, 1); YTL = RTL(:, 2); ZTL = RTL(:, 3);

    % Pelvis (rood/groen/blauw)
    quiver3(origin(1), origin(2), origin(3), scale*XP(1), scale*XP(2), scale*XP(3), 'r', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*YP(1), scale*YP(2), scale*YP(3), 'g', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*ZP(1), scale*ZP(2), scale*ZP(3), 'b', 'LineWidth', 1.5);

    % Thigh (magenta/cyaan/zwart)
    quiver3(origin(1), origin(2), origin(3), scale*XTL(1), scale*XTL(2), scale*XTL(3), 'm', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*YTL(1), scale*YTL(2), scale*YTL(3), 'c', 'LineWidth', 1.5);
    quiver3(origin(1), origin(2), origin(3), scale*ZTL(1), scale*ZTL(2), scale*ZTL(3), 'k', 'LineWidth', 1.5);
end

legend('Pelvis X','Pelvis Y','Pelvis Z','Thigh X','Thigh Y','Thigh Z');



%% Rotatie matrices & Euler/Cardan angles

amount_frames = height(filtered_data);
R_UT = zeros(amount_frames,3,3); 
for i = 1:amount_frames
    RU = squeeze(U(i,:,:)); % 3x3 upper arm
    RT = squeeze(T(i,:,:)); % 3x3 thorax
    RF = squeeze(F(i,:,:)); % 3x3 forearm
    RP = squeeze(P(i,:,:)); % 3x3 pelvic
    
    % Relatieve matrix: Upper arm relative to Thorax
    R_rel_UT = RT.' * RU; 

    % Relatieve matrix: Forarm relative to Upper arm
    R_rel_FU = RU.' * RF;

    % Relatieve matrix: Thorax relative to Pelvic
    R_rel_TP = RP.' * RT;

    % Relatieve matrix: Shank Left relative to Thigh Left

    % Euler-hoeken voor Shoulder motion based on R_rel_UT (ISB: Y-X-Y volgorde)
    % met Y = Ythorax, X = Xhumerus, Y = Yhumerus
    %euler_rad = rotm2eul(R_rel_UT, 'YXY');  % [gamma beta alpha] in radialen
    %euler_shoulder_rad(i,:) = euler_rad;
    Yt = RT(:,2);  % 2e kolom van Thorax = Y-as thorax
    Xh = RU(:,1);  % 1e kolom van Upper arm = X-as humerus
    Yh = RU(:,2);  % 2e kolom van Upper arm = Y-as humerus
    
    euler_angles_shoulder_deg = computeEulerFromAxes(R_rel_UT, Yt, Xh, Yh);  
    euler_shoulder_deg(i,:) = euler_angles_shoulder_deg;
    % unwrap de euler angles om sprongen van 180 graden weg te filteren
    euler_shoulder_deg_unwrapped = unwrapEulerAngles(euler_shoulder_deg);

    
    % Euler-hoeken voor Elbow motion based on R_rel_FU (ISB: Z-X-Y volgorde)
    % met Z = Zhumerus, X = Xforearm (loodrecht op Z en Y), Y = Yforearm
    Zh = RU(:,3);
    Xf = RF(:,1);
    Yf = RF(:,2);
    
    euler_angles_elbow_deg = computeEulerFromAxes(R_rel_FU, Zh, Xf, Yf);
    euler_elbow_deg(i,:) = euler_angles_elbow_deg;
    euler_elbow_deg_unwrapped = unwrapEulerAngles(euler_elbow_deg);


    % Euler-hoeken voor Core motion based on R_rel_TP (ISB: ... volgorde)

    % Euler-hoeken voor Pelvis motion within global frame based on
    % att_mat_P (ISB: ... volgorde)

    % Euler-hoeken voor Thorax motion within global frame based on
    % att_mat_T (ISB: ... volgorde)

    % Euler-hoeken voor Left Knee motion based on R_rel_STL (ISB: ... volgorde)

    
end

    % unwrap de euler angles
    %euler_shoulder_rad_unwrapped = unwrap(euler_shoulder_rad);
    %euler_shoulder_rad_unwrapped = unwrap(ShoulderAnglesISB);

    % Check of de rotatiematrices orthonormaal zijn, dit is vereist voor de
    % rotm2eul functie
    orthonormaal = norm(R_rel_UT * R_rel_UT.' - eye(3)) < 1e-6; % moet true zijn
    determinant = det(R_rel_UT); % moet dicht bij 1 liggen
    disp(orthonormaal);
    disp(determinant);

disp('R_rel_UT (Upper arm relative to Thorax) aangemaakt.');
disp('R_rel_FU (Forearm relative to Upper arm) aangemaakt.');


%% EULER/CARDAN angles in degrees

% SHOULDER
% Optioneel opsplitsen
gammaS  = euler_shoulder_deg_unwrapped(:,1);  % plane of elevation
betaS   = euler_shoulder_deg_unwrapped(:,2);  % elevation
alphaS  = euler_shoulder_deg_unwrapped(:,3);  % axial rotation

% Struct voor overzicht
ShoulderAngles = table(gammaS, betaS, alphaS,'VariableNames', {'PlaneOfElevation_deg','Elevation_deg','AxialRotation_deg'});

% Toon eerste paar waarden
disp('Eerste 10 rijen van de schouderhoeken (Euler/Cardan):');
disp(ShoulderAngles(1:100,:));

% ELBOW
gammaE = euler_elbow_deg_unwrapped(:,1);
betaE  = euler_elbow_deg_unwrapped(:,2);
alphaE = euler_elbow_deg_unwrapped(:,3);
ElbowAngles = table(gammaE, betaE, alphaE,'VariableNames', {'Flexion/Extension_deg','Carrying_angle_deg,','AxialRotation_deg'});
disp('Eerste 10 rijen van de ellebooghoeken (Euler/Cardan):');
disp(ElbowAngles(1:100,:));

%% Ball Release
PLR = [filtered_data.PLRX, filtered_data.PLRY, filtered_data.PLRZ];
fs = 300;                 % Sampling frequency in Hz
dt = 1 / fs;              % Time step
N = size(PLR, 1);         % Number of frames
t = (0:N-1) * dt;         % Time vector in seconds
FC_index = 427;

vel = gradient(PLR, dt);
acc = gradient(vel, dt);

speed = vecnorm(vel, 2, 2);  % Euclidean norm per row

post_FC_speed = speed(FC_index:end);
[~, idx_max_speed] = max(post_FC_speed);

BR_index = FC_index - 1 + idx_max_speed;
BR_time = t(BR_index);

figure;
subplot(3,1,1); plot(t, PLR); title('PLR Position'); legend('X','Y','Z');
subplot(3,1,2); plot(t, vel); title('PLR Velocity'); legend('Vx','Vy','Vz');
subplot(3,1,3); plot(t, speed); title('PLR Speed');
hold on; plot(BR_time, speed(BR_index), 'ro'); legend('Speed', 'Ball Release');

figure;
plot3(PLR(:,1), PLR(:,2), PLR(:,3), 'b-'); hold on;
plot3(PLR(BR_index,1), PLR(BR_index,2), PLR(BR_index,3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
title('3D Trajectory of PLR'); xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;
legend('Trajectory', 'Ball Release');

%%
% ======================================
% 3D KINEMATICA – PERSOON 3 TEMPLATE
% Snelheden, Versnellingen & CRP
% ======================================

% %% 1. Inlezen van Hoekdata (van Persoon 2)
% % - Voeg hier code toe om de hoekdata in te lezen
% % - Denk aan struct of matrixvorm, per gewricht
% 
% % voorbeeldstructuur:
% % angles_shoulder = ...;
% % angles_elbow = ...;
% % angles_core = ...;
% 
% %% 2. Rotatiesnelheden Berekenen
% % - Bereken eerste afgeleide van de hoeken
% % - Voeg filtering toe indien nodig
% 
% % Voor snelheid:
% vel_shoulder = diff(angles_shoulder) * fs;  % fs = sampling frequentie (300 Hz)
% vel_elbow = diff(angles_elbow) * fs;
% vel_core = diff(angles_core) * fs;
% 
% 
% %% 3. Rotatieversnellingen Berekenen
% % - Bereken tweede afgeleide van de hoeken
% 
% % Voor versnelling:
% acc_shoulder = diff(vel_shoulder) * fs;
% acc_elbow = diff(vel_elbow) * fs;
% acc_core = diff(vel_core) * fs;
% 
% 
% %% 4. CRP Methode 1: Hoek-Snelheid Methode
% % Normaliseren van hoeken en snelheden:
% % manier 1: (+-1)
% shoulder_norm = (angles_shoulder - min(angles_shoulder)) / (max(angles_shoulder) - min(angles_shoulder)) * 2 - 1;
% elbow_norm = (angles_elbow - min(angles_elbow)) / (max(angles_elbow) - min(angles_elbow)) * 2 - 1;
% vel_shoulder_norm = (vel_shoulder - min(vel_shoulder)) / (max(vel_shoulder) - min(vel_shoulder)) * 2 - 1;
% vel_elbow_norm = (vel_elbow - min(vel_elbow)) / (max(vel_elbow) - min(vel_elbow)) * 2 - 1;
% 
% % manier 2: z-score
% shoulder_norm = (angles_shoulder - mean(angles_shoulder)) / std(angles_shoulder);
% elbow_norm = (angles_elbow - mean(angles_elbow)) / std(angles_elbow);
% vel_shoulder_norm = (vel_shoulder - mean(vel_shoulder)) / std(vel_shoulder);
% vel_elbow_norm = (vel_elbow - mean(vel_elbow)) / std(vel_elbow);
% 
% % - Bereken fasehoeken en CRP
% phase_angle_shoulder = atan2(vel_shoulder_norm, shoulder_norm);  % in radialen
% phase_angle_elbow = atan2(vel_elbow_norm, elbow_norm);  % in radialen
% 
% crp1 = crp_angle_velocity(angle1, angle2, vel1, vel2);
% 
% %% 5. CRP Methode 2: Hilbert Methode
% % - Gebruik Hilbert transform om fasen te verkrijgen
% % - Bereken CRP op basis van deze fasen
% 
% % crp2_shoulder = ...;
% 
% %% 6. Vergelijking CRP-methodes
% % - Plot of analyse van verschillen tussen beide methodes
% 
% % figuren of statistieken ...
% 
% %% 7. Export / Output
% % - Struct klaarzetten voor Persoon 4
% % - Opslaan van CRP-resultaten en afgeleiden
% 
% % save('output_persoon3.mat', ...)

function [Euler_shoulder, Euler_elbow, Euler_core, Euler_pelvis, Euler_knee, Euler_thorax] = computeEulerAngles(amount_frames, U, T, P, TL, SL, F)
% COMPUTEEULERANGLES Compute Euler angles (in degrees) from relative rotation matrices
%
%   Inputs:
%   U, T, TL : Nx3x3 attitude matrices of UpperArm, Thorax, Pelvis, ThighLeft, Shankleft, Forearm respectively
%
%   Outputs (each Nx3):
%   Euler_shoulder : [plane_of_elevation, elevation, axial_rotation] (ISB Y-X-Y)
%   Euler_elbow : [flexion/extension, carrying_angle, axial_rotation] (ISB Z-X-Y)
%   Euler_core : [α, β, γ] or Thorax relative to Pelvis in XYZ order
%   Euler_pelvis : [α, β, γ] of Pelvis in global frame (e.g. ZYX)
%   Euler_knee : [α, β, γ] of ShankLeft relative to ThighLeft in XYZ
%

% Arrays voor Euler-angles
euler_shoulder_deg = zeros(amount_frames, 3);
euler_elbow_deg    = zeros(amount_frames, 3);
euler_core_deg     = zeros(amount_frames, 3);
euler_pelvis_deg   = zeros(amount_frames, 3);
euler_thorax_deg   = zeros(amount_frames, 3);
euler_LKnee_deg    = zeros(amount_frames, 3);

  for i = 1:amount_frames
    % squeeze haalt de overbodige dimensie weg zodat je een 3x3 matrix krijgt
    % U(i,:,:) heeft vorm [1,3,3] en wij hebben [3,3] nodig
    RU = squeeze(U(i,:,:)); % 3x3 upper arm
    RT = squeeze(T(i,:,:)); % 3x3 thorax
    RF = squeeze(F(i,:,:)); % 3x3 forearm
    RP = squeeze(P(i,:,:)); % 3x3 pelvic
    RTL = squeeze(TL(i,:,:)); % 3x3 left thigh
    RSL = squeeze(SL(i,:,:)); % 3x3 left shank

    % Relatieve matrix: R_rel = Rbase.' * Rsegment
    % berekent 
    R_rel_UT = RT.' * RU; % Relatieve matrix: Upper arm relative to Thorax
    R_rel_FU = RU.' * RF; % Relatieve matrix: Forarm relative to Upper arm
    R_rel_TP = RP.' * RT; % Relatieve matrix: Thorax relative to Pelvic
    R_rel_STL = RTL.' * RSL; % Relatieve matrix: Shank Left relative to Thigh Left

  
    % SHOULDER
    % Euler-hoeken voor Shoulder motion based on R_rel_UT (ISB: Y-X-Y volgorde)
    % met Y = Ythorax, X = Xhumerus, Y = Yhumerus
    %euler_rad = rotm2eul(R_rel_UT, 'YXY');  % [gamma beta alpha] in radialen
    %euler_shoulder_rad(i,:) = euler_rad;
    Yt = RT(:,2);  % 2e kolom van Thorax = Y-as thorax
    Xh = RU(:,1);  % 1e kolom van Upper arm = X-as humerus
    Yh = RU(:,2);  % 2e kolom van Upper arm = Y-as humerus
    euler_shoulder_deg(i,:) = computeEulerFromAxes(R_rel_UT, Yt, Xh, Yh);

    % ELBOW
    % Euler-hoeken voor Elbow motion based on R_rel_FU (ISB: Z-X-Y volgorde)
    % met Z = Zhumerus, X = Xforearm (loodrecht op Z en Y), Y = Yforearm
    Zh = RU(:,3);
    Xf = RF(:,1);
    Yf = RF(:,2);
    euler_elbow_deg(i,:) = computeEulerFromAxes(R_rel_FU, Zh, Xf, Yf);

    % CORE
    % Euler-hoeken voor Core motion based on R_rel_TP (ISB: geen volgorde gegeven)
    Xp = RP(:,1);
    Yp = RP(:,2);
    Zp = RP(:,3);
    euler_core_deg(i,:) = computeEulerFromAxes(R_rel_TP, Xp, Yp, Zp); 

    % PELVIS
    % Euler-hoeken voor Pelvis motion within global frame based on
    % att_mat_P (ISB: ... volgorde)
    % Hier kunnnen we de rotm2eul functie gebruiken, omdat we tov het
    % globale coordinatensysteem kijken, we nemen XYZ volgorde
    euler_pelvis_rad = rotm2eul(RP, 'XYZ');
    euler_pelvis_deg(i,:) = rad2deg(euler_pelvis_rad);

    % THORAX
    % Euler-hoeken voor Thorax motion within global frame based on
    % att_mat_T (ISB: ... volgorde)
    % Hier kunnnen we de rotm2eul functie gebruiken, omdat we tov het
    % globale coordinatensysteem kijken, we nemen XYZ volgorde
    euler_thorax_rad = rotm2eul(RT, 'XYZ');
    euler_thorax_deg(i,:) = rad2deg(euler_thorax_rad);

    % LEFT KNEE
    % Euler-hoeken voor Left Knee motion based on R_rel_STL (ISB: ... volgorde)
    Xtl = RTL(:,1);
    Ytl = RTL(:,2);
    Ztl = RTL(:,3);
    euler_LKnee_deg(i,:) = computeEulerFromAxes(R_rel_STL, Xtl, Ytl, Ztl);

end
% Unwrap Euler-angles om sprongen weg te halen
Euler_shoulder = unwrapEulerAngles(euler_shoulder_deg);
Euler_elbow    = unwrapEulerAngles(euler_elbow_deg);
Euler_core     = unwrapEulerAngles(euler_core_deg);
Euler_pelvis   = unwrapEulerAngles(euler_pelvis_deg);
Euler_thorax   = unwrapEulerAngles(euler_thorax_deg);
Euler_knee    = unwrapEulerAngles(euler_LKnee_deg);

disp('Relative rotation matrices generated')
disp('Euler angles calculated');
end

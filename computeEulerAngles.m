function [Euler_shoulder, Euler_elbow, Euler_core, Euler_pelvis, Euler_knee] = computeEulerAngles(R_rel_UT, R_rel_FU, R_rel_TP, R_rel_STL, U, T, P, TL, F)
% COMPUTEEULERANGLES Compute Euler angles (in degrees) from relative rotation matrices
%
%   Inputs:
%   R_rel_UT : Nx3x3 array (UpperArm relative to Thorax)
%   R_rel_FU : Nx3x3 array (Forearm relative to UpperArm)
%   R_rel_TP : Nx3x3 array (Thorax relative to Pelvis)
%   R_rel_STL : Nx3x3 array (ShankLeft relative to ThighLeft)
%   U, T, TL : Nx3x3 arrays of UpperArm, Thorax, ThighLeft respectively
%
%   Outputs (each Nx3):
%   Euler_shoulder : [plane_of_elevation, elevation, axial_rotation] (ISB Y-X-Y)
%   Euler_elbow : [flexion/extension, carrying_angle, axial_rotation] (ISB Z-X-Y)
%   Euler_core : [α, β, γ] or Thorax relative to Pelvis in XYZ order
%   Euler_pelvis : [α, β, γ] of Pelvis in global frame (e.g. ZYX)
%   Euler_knee : [α, β, γ] of ShankLeft relative to ThighLeft in XYZ
%
 N = size(R_rel_UT,1);
  Euler_shoulder = zeros(N,3);
  Euler_elbow    = zeros(N,3);
  Euler_core     = zeros(N,3);
  Euler_pelvis   = zeros(N,3);
  Euler_knee     = zeros(N,3);

  for i = 1:N
    % 1) Shoulder (ISB Y-X-Y)
    Rt_UT = squeeze(R_rel_UT(i,:,:));  % Thorax' * UpperArm
    RU   = squeeze(U(i,:,:));          % 3×3 UpperArm
    RT   = squeeze(T(i,:,:));          % 3×3 Thorax
    % Define the three axis vectors:
    Yt = RT(:,2);  % Ythorax = second column of Thorax
    Xh = RU(:,1);  % Xhumerus
    Yh = RU(:,2);  % Yhumerus
    angles_s = computeEulerFromAxes(Rt_UT, Yt, Xh, Yh);
    Euler_shoulder(i,:) = unwrapEulerAngles(angles_s);

    % 2) ELBOWe (ISB Z-X-Y)
    Rt_FU = squeeze(R_rel_FU(i,:,:));  % UpperArm' * Forearm
    Zh = RU(:,3);                     % Zhumerus
    RF = squeeze(F(i,:,:));           % 3×3 Forearm
    Xf = RF(:,1);                     % Xforearm
    Yf = RF(:,2);                     % Yforearm
    angles_e = computeEulerFromAxes(Rt_FU, Zh, Xf, Yf);
    Euler_elbow(i,:) = unwrapEulerAngles(angles_e);

    % 3) CORE (Thorax relatief aan Pelvis). We kiezen hier de ‘XYZ’-volgorde:
    Rt_TP = squeeze(R_rel_TP(i,:,:));  % Pelvis' * Thorax
    angles_c = rotm2eul(Rt_TP, 'XYZ');  % standaard extrinsiek XYZ
    Euler_core(i,:) = rad2deg(angles_c);

    % 4) PELVIS in de globale wereld (attitude P):
    Rp = squeeze(P(i,:,:));             % 3×3 Pelvis
    angles_p = rotm2eul(Rp, 'ZYX');      % gebruik ZYX (intrinsiek) of een andere conventie
    Euler_pelvis(i,:) = rad2deg(angles_p);

    % 5) KNIE (ShankLeft relatief aan ThighLeft), standaard ‘XYZ’:
    Rk = squeeze(R_rel_STL(i,:,:));     % ThighLeft' * ShankLeft
    angles_k = rotm2eul(Rk, 'XYZ');
    Euler_knee(i,:) = rad2deg(angles_k);
  end
end

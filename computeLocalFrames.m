function [F, U, T, P, TL, SL] = computeLocalFrames(filtered_data)
%
% COMPUTELOCALFRAMES calculates all local coordinat systems for all segments
% filtered_data: table with filtered marker coordinates.
% Expected columns: MLLX, MLLY, MLLZ, MMLX, MMLY, MMLZ, CLLX, … and so on.
%
% Outputs (according to ISB): 
%   F   : Nx3x3 local coordinate system Forearm Right
%   U   : Nx3x3 local coordinate system Upper Arm Right
%   T   : Nx3x3 local coordinate system Thorax
%   P   : Nx3x3 local coordinate system Pelvis
%   TL  : Nx3x3 local coordinate system Left Thigh
%   SL  : Nx3x3 local coordinate system Left Shank
%         (N = number of frames, in our project it will be 558)

  N = height(filtered_data);

  % Pre-allocation
  F  = zeros(N,3,3);
  U  = zeros(N,3,3);
  T  = zeros(N,3,3);
  P  = zeros(N,3,3);
  TL = zeros(N,3,3);
  SL = zeros(N,3,3);

  % Marker-arrays
  PLR = [filtered_data.PLRX, filtered_data.PLRY, filtered_data.PLRZ];
  PMR = [filtered_data.PMRX, filtered_data.PMRY, filtered_data.PMRZ];
  ELR = [filtered_data.ELRX, filtered_data.ELRY, filtered_data.ELRZ];
  EMR = [filtered_data.EMRX, filtered_data.EMRY, filtered_data.EMRZ];
  AR  = [filtered_data.ARX,  filtered_data.ARY,  filtered_data.ARZ];
  PX  = [filtered_data.PXX,  filtered_data.PXY,  filtered_data.PXZ];
  T7  = [filtered_data.T7X,  filtered_data.T7Y,  filtered_data.T7Z];
  MS  = [filtered_data.MSX,  filtered_data.MSY,  filtered_data.MSZ];
  C7  = [filtered_data.C7X,  filtered_data.C7Y,  filtered_data.C7Z];
  SIASL = [filtered_data.SIASLX, filtered_data.SIASLY, filtered_data.SIASLZ];
  SIASR = [filtered_data.SIASRX, filtered_data.SIASRY, filtered_data.SIASRZ];
  SIPSL = [filtered_data.SIPSLX, filtered_data.SIPSLY, filtered_data.SIPSLZ];
  SIPSR = [filtered_data.SIPSRX, filtered_data.SIPSRY, filtered_data.SIPSRZ];
  HL  = [filtered_data.HLX,  filtered_data.HLY,  filtered_data.HLZ];
  CLL = [filtered_data.CLLX, filtered_data.CLLY, filtered_data.CLLZ];
  CML = [filtered_data.CMLX, filtered_data.CMLY, filtered_data.CMLZ];
  MLL = [filtered_data.MLLX, filtered_data.MLLY, filtered_data.MLLZ];
  MML = [filtered_data.MMLX, filtered_data.MMLY, filtered_data.MMLZ];

  for i = 1:N
    % 1) Forearm Right (F)
    % Y = PLR to mid elbow
    midpoint_elbow = 0.5*(ELR(i,:)+EMR(i,:));
    Yf = (midpoint_elbow - PLR(i,:));
    % X = perpendicular to plane PLR-PMR center
    v1 = PLR(i,:) - PMR(i,:);
    v2 = midpoint_elbow - PMR(i,:);
    Xf = (cross(v1,v2));
    Zf = cross(Xf, Yf);
    F(i,:,:) = [ Unity(Xf); Unity(Yf); Unity(Zf) ]';
    % Check norm of every axis, make sure we dont have zero vectors
    if norm(Xf) < 1e-3 || norm(Yf) < 1e-3 || norm(Zf) < 1e-3
        warning("Frame %d: nearly collinear forearm axes", i);
    end

    % 2) Upper Arm Right (U)
    midpoint_elbow = 0.5*(ELR(i,:)+EMR(i,:));
    Yu = (AR(i,:) - midpoint_elbow);
    % Y-axis Forearm
    Yf = squeeze(F(i,:,2));  
    Zu = cross(Yu, Yf);
    Xu = cross(Yu, Zu);
    U(i,:,:) = [ Unity(Xu); Unity(Yu); Unity(Zu) ]';
    if norm(Xu) < 1e-3 || norm(Yu) < 1e-3 || norm(Zu) < 1e-3
        warning("Frame %d: nearly collinear forearm axes", i);
    end
    % 3) Thorax (T)
    lower = 0.5*(PX(i,:)+T7(i,:));
    upper = 0.5*(MS(i,:)+C7(i,:));
    Yt = (upper - lower);
    v1 = C7(i,:) - MS(i,:);
    v2 = lower  - MS(i,:);
    Zt = (cross(v1,v2));
    Xt = cross(Yt,Zt);
    T(i,:,:) = [ Unity(Xt); Unity(Yt); Unity(Zt) ]';
    if norm(Xt) < 1e-3 || norm(Yt) < 1e-3 || norm(Zt) < 1e-3
        warning("Frame %d: nearly collinear forearm axes", i);
    end
    % 4) Pelvis (P)
    midsips = 0.5*(SIPSR(i,:)+SIPSL(i,:));
    Zp = (SIASL(i,:) - SIASR(i,:));
    plane = cross(midsips - SIASL(i,:), SIASR(i,:) - SIASL(i,:));
    Xp = cross(Zp, plane);
    Yp = cross(Xp, Zp);
    P(i,:,:) = [ Unity(Xp); Unity(Yp); Unity(Zp) ]';
    if norm(Xp) < 1e-3 || norm(Yp) < 1e-3 || norm(Zp) < 1e-3
        warning("Frame %d: nearly collinear forearm axes", i);
    end
    % 5) Left Thigh (TL)
    knee_mid = 0.5*(CLL(i,:)+CML(i,:));
    Ytl = (HL(i,:) - knee_mid);
    tempZ = cross(CML(i,:) - HL(i,:), CLL(i,:) - HL(i,:));
    Ztl   = cross(Ytl, tempZ);
    Xtl   = cross(Ztl, Ytl);
    TL(i,:,:) = [ Unity(Xtl); Unity(Ytl); Unity(Ztl) ]';
    if norm(Xtl) < 1e-3 || norm(Ytl) < 1e-3 || norm(Ztl) < 1e-3
        warning("Frame %d: nearly collinear forearm axes", i);
    end
    % 6) Left Shank (SL)
    origin  = 0.5*(MLL(i,:)+MML(i,:));
    Zsl     = (MLL(i,:) - MML(i,:));
    v1 = CLL(i,:) - MML(i,:);
    v2 = CML(i,:) - MLL(i,:);
    Xsl = cross(v1,v2);
    Ysl = cross(Xsl, Zsl);
    Xsl = cross(Ysl, Zsl);
    SL(i,:,:) = [ Unity(Xsl); Unity(Ysl); Unity(Zsl) ]';
    if norm(Xsl) < 1e-3 || norm(Ysl) < 1e-3 || norm(Zsl) < 1e-3
        warning("Frame %d: nearly collinear forearm axes", i);
    end
  end

% Smoothing attitude matrix F
for i = 1:3
    for j = 1:3
        F(:,i,j) = smoothdata(F(:,i,j), 'movmean', 60);  
    end
end
% Smoothing attitude matrix U 
for i = 1:3
    for j = 1:3
        F(:,i,j) = smoothdata(F(:,i,j), 'movmean', 60); 
    end
end
% Keep continuity of orientation 
F = fixAttitudeContinuity(F);
U = fixAttitudeContinuity(U);
T = fixAttitudeContinuity(T);
P = fixAttitudeContinuity(P);
TL = fixAttitudeContinuity(TL);
SL = fixAttitudeContinuity(SL);
end


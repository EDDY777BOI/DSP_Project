function [R_rel_UT, R_rel_FU, R_rel_TP, R_rel_STL] = computeRelativeRotations(U, F, T, P, TL, SL)
%COMPUTERELATIVEROTATIONS Calculate the relative rotation matrices between segments
%   Detailed explanation:
%   R_rel_UT  = Thorax' * UpperArm
%   R_rel_FU  = UpperArm' * Forearm
%   R_rel_TP  = Pelvis' * Thorax
%   R_rel_STL = ThighLeft' * ShankLeft
%
%   Input: 
%   U, F, T, P, TL, SL are Nx3x3 arrays (as from computeLocalFrames).
%
%   Output:
%   R_rel_UT, R_rel_FU, R_rel_TP, R_rel_STL are each Nx3x3 arrays.
N = size(U,1);
  R_rel_UT  = zeros(N,3,3);
  R_rel_FU  = zeros(N,3,3);
  R_rel_TP  = zeros(N,3,3);
  R_rel_STL = zeros(N,3,3);

  for i = 1:N
    RU = squeeze(U(i,:,:));
    RF = squeeze(F(i,:,:));
    RT = squeeze(T(i,:,:));
    RP = squeeze(P(i,:,:));
    RTL= squeeze(TL(i,:,:));
    RSL= squeeze(SL(i,:,:));

    % Upper arm relative to Thorax
    R_rel_UT(i,:,:) = RT.' * RU;

    % Forearm relative to Upper arm
    R_rel_FU(i,:,:) = RU.' * RF;

    % Thorax relative to Pelvis
    R_rel_TP(i,:,:) = RP.' * RT;

    % ShankLeft relative to ThighLeft
    R_rel_STL(i,:,:) = squeeze(TL(i,:,:)).' * squeeze(SL(i,:,:));
    % Let op: TL(i,:,:) = 3×3 Thigh; SL(i,:,:) = 3×3 Shank
    %       R_rel_STL = TL' * SL
  end
end


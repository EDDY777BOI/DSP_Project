function [BR_index, BR_time] = computeBallRelease(filtered_data, FC_index, fs)
% COMPUTEBALLRELEASE Calculates index and time of Ball Release (max velocity)
% filtered_data : table with filtered marker coordinates
% FC_index : frame in which Foot Contact occurred
% fs : sample rate
%
% Outputs:
% BR_index : frame index in which PLR velocity is highest after FC
% BR_time : time in seconds (BR_index / fs)

% 1) Marker PLR (punten PLR X, Y, Z)
  PLR = [filtered_data.PLRX, filtered_data.PLRY, filtered_data.PLRZ];

  % 2) Tijd‐vector
  N = size(PLR,1);
  dt = 1/fs;
  t  = (0:N-1)' * dt;

  % 3) Snelheden (component‐gewijs via gradient)
  vel = gradient(PLR, dt);        % Nx3 matrix (Vx, Vy, Vz)

  % 4) Snelheidsnorm (Euclidische norm per rij)
  speed = vecnorm(vel, 2, 2);     % Nx1 vector

  % 5) Zoek maximum van “speed” ná FC_index
  postRange = FC_index : N;
  [~, relIdx] = max(speed(postRange));
  BR_index = postRange(relIdx);
  BR_time  = t(BR_index);
end


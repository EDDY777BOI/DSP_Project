function FC_index = computeFootContactLeftLeg(filtered_data, window, fs)
%COMPUTEFOOTCONTACTLEFTLEG Calculates frame of foot contact left
% filtered_data : table with filtered markers
% window : row vector with frame indices (410:470)
% fs : sample rate (300)
%
% FC_index : the frame (index) in which foot contact occurs
dt = 1/fs;

  % 1) X-positie van malleoli (gemiddelde van MLL en MML)
  MLL = [filtered_data.MLLX, filtered_data.MLLY, filtered_data.MLLZ];
  MML = [filtered_data.MMLX, filtered_data.MMLY, filtered_data.MMLZ];
  x_mal = 0.5*(MLL(:,1) + MML(:,1));  % gemiddeld x

  % 2) Snelheid & acceleratie in X
  vel_mal = [0; diff(x_mal)/dt];
  acc_mal = [0; diff(vel_mal)/dt];

  % 3) Zoek de eerste piek (abs) van acc in het opgegeven window
  sub_acc = abs(acc_mal(window));
  [~, idx_rel] = max(sub_acc);   % idx_rel: index binnen het “window”-vector
  FC_index = window(idx_rel);
end


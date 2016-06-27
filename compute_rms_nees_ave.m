function [rms, nees] = compute_rms_nees_ave( rmsall, neesall )
% compute rms and nees from multiple tests

nsteps = size(rmsall.position, 2);
ntests = size(rmsall.position, 1);

% compute rms
rmspos_sum = sum(rmsall.position.^2, 1);
rms.position = sqrt(rmspos_sum/ntests);

rmsori_sum = sum(rmsall.orientation.^2, 1);
rms.orientation = sqrt(rmsori_sum/ntests);

% compute nees
neesall_pose = neesall.pose(:, 2:end);
neesall_orientation = neesall.orientation(:, 2:end);

neespose_sum = sum(neesall_pose, 1);
nees.pose = neespose_sum/ntests;

neesorientation_sum = sum(neesall_orientation, 1);
nees.orientation = neesorientation_sum/ntests;





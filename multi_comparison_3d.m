% comparison of following methods in 3D environment
% - FEKF SLAM 
% - FEJ-EKF SLAM
% - EKF SLAM
clear;
clc;
close all;
% add directories
addpath('datagen_3d/');
addpath('f_ekf_3d/');
addpath('right_ekf_3d/');
addpath('lie_utils/');
addpath('ekf_3d/');
% generate simulation data
niter = 1;
nsteps = 400;
REKF_RMS.position = [];
REKF_RMS.orientation = [];
REKF_NEES.pose = [];
REKF_NEES.orientation = [];

FEKF_RMS.position = [];
FEKF_RMS.orientation = [];
FEKF_NEES.pose = [];
FEKF_NEES.orientation = [];



EKF_RMS.position = [];
EKF_RMS.orientation = [];
EKF_NEES.pose = [];
EKF_NEES.orientation = [];

for i = 1:niter
    tic;
    data = gen_data( 0 );
    save(['data/', int2str(i)], 'data');
    nposes = size(data.poses.position, 2);
    nlandmarks = size(data.landmarks, 1);
    fprintf('Generate %d poses and %d landmarks\n', nposes, nlandmarks);
    
    REKF_result = REKF_SLAM( data );
    [REKF_RMS.position(i, :), REKF_RMS.orientation(i, :), REKF_NEES.pose(i, :), REKF_NEES.orientation(i, :)] = ...
        REKF_plot_rms_nees( REKF_result, data, 0 );
    
    FEKF_result = FEKF_SLAM( data );
    [FEKF_RMS.position(i, :), FEKF_RMS.orientation(i, :), FEKF_NEES.pose(i, :), FEKF_NEES.orientation(i, :)] = ...
        FEKF_plot_rms_nees( FEKF_result, data, 0 );
    

    EKF_result = EKF_SLAM( data );
    [EKF_RMS.position(i, :), EKF_RMS.orientation(i, :), EKF_NEES.pose(i, :), EKF_NEES.orientation(i, :)] = ...
        EKF_plot_rms_nees( EKF_result, data, 0 );
    toc;
    
end

[REKF_RMS_ave, REKF_NEES_ave] = compute_rms_nees_ave( REKF_RMS, REKF_NEES );
[FEKF_RMS_ave, FEKF_NEES_ave] = compute_rms_nees_ave( FEKF_RMS, FEKF_NEES );
[EKF_RMS_ave, EKF_NEES_ave] = compute_rms_nees_ave( EKF_RMS, EKF_NEES );

figure;
%subplot(2,2,1);
plot(1:size(REKF_RMS_ave.position, 2), REKF_RMS_ave.position, 'r'); hold on;
plot(1:size(FEKF_RMS_ave.position, 2), FEKF_RMS_ave.position, 'g'); hold on;
plot(1:size(EKF_RMS_ave.position, 2), EKF_RMS_ave.position, 'k'); hold on;
legend('R-EKF', 'FEJ-EKF', 'EKF', 'Location','northeast');
title('RMS:position(meter)');xlim([0,size(REKF_RMS_ave.position, 2)]);

figure;
plot(1:size(REKF_RMS_ave.orientation, 2), REKF_RMS_ave.orientation, 'r'); hold on;
plot(1:size(FEKF_RMS_ave.orientation, 2), FEKF_RMS_ave.orientation, 'g'); hold on;
plot(1:size(EKF_RMS_ave.orientation, 2), EKF_RMS_ave.orientation, 'k'); hold on;
legend('R-EKF', 'FEJ-EKF',  'EKF', 'Location','northeast');
title('RMS:orientation(rad)');xlim([0,size(REKF_RMS_ave.orientation, 2)]);

figure;
semilogy(1:size(REKF_NEES_ave.orientation, 2), REKF_NEES_ave.orientation, 'r'); hold on;
semilogy(1:size(FEKF_NEES_ave.orientation, 2), FEKF_NEES_ave.orientation, 'g'); hold on;
semilogy(1:size(EKF_NEES_ave.orientation, 2), EKF_NEES_ave.orientation, 'k'); hold on;
legend('R-EKF', 'FEJ-EKF', 'EKF', 'Location','northeast');
title('NEES:orientation');xlim([0,size(REKF_NEES_ave.orientation, 2)]);

figure;
semilogy(1:size(REKF_NEES_ave.pose, 2), REKF_NEES_ave.pose, 'r'); hold on;
semilogy(1:size(FEKF_NEES_ave.pose, 2), FEKF_NEES_ave.pose, 'g'); hold on;
semilogy(1:size(EKF_NEES_ave.pose, 2), EKF_NEES_ave.pose, 'k'); hold on;
legend('R-EKF', 'FEJ-EKF', 'EKF', 'Location','northeast');
title('NEES:pose');xlim([0,size(REKF_NEES_ave.pose, 2)]);

fprintf('Mean values:\n');
fprintf('           RMS-Position    RMS-Orientation     NEES-Pose   NEES-Orientation\n');
fprintf('R-EKF:          %.5f            %.5f       %.5f           %.5f\n', ...
    mean(REKF_RMS_ave.position), mean(REKF_RMS_ave.orientation), mean(REKF_NEES_ave.pose), mean(REKF_NEES_ave.orientation));
fprintf('FEJ-EKF:          %.5f            %.5f       %.5f           %.5f\n', ...
    mean(FEKF_RMS_ave.position), mean(FEKF_RMS_ave.orientation), mean(FEKF_NEES_ave.pose), mean(FEKF_NEES_ave.orientation));
fprintf('EKF:            %.5f            %.5f      %.5f          %.5f\n', ...
    mean(EKF_RMS_ave.position), mean(EKF_RMS_ave.orientation), mean(EKF_NEES_ave.pose), mean(EKF_NEES_ave.orientation));


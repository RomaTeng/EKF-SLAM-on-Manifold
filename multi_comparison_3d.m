% comparison of following methods in 3D environment
% - FEKF SLAM 
% - FEJ-EKF SLAM
% - EKF SLAM
clear;
clc;
close all;
% add directories
addpath('datagen_3d/');
addpath('f_ekf_3dTest_mod/');
addpath('se3_ekf_3d/');
addpath('robotcentric_ekf_3d_mod/');
addpath('right_ekf_3d_mod/');
addpath('not_right_ekf_3d/');
addpath('ekf_3d_mod/');
addpath('lie_utils/');

% generate simulation data
niter = 100;
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
    if max(data.state(:,4) )<499.9
       data = gen_data( 0 );
    end        
    
    
    
    save(['data/', int2str(i)], 'data');
    nposes = size(data.poses.position, 2);
    nlandmarks = size(data.landmarks, 1);
    fprintf('Generate %d poses and %d landmarks\n', nposes, nlandmarks);
    
    REKF_result = REKF_SLAM( data );
    fprintf('R-EKF:\n, it is %d -th loop',i);
    [REKF_RMS.position(i, :), REKF_RMS.orientation(i, :), REKF_NEES.pose(i, :), REKF_NEES.orientation(i, :)] = ...
        REKF_plot_rms_nees( REKF_result, data, 0 );
    
    
    
    FEKF_result = FEKF_SLAM( data );
        fprintf('F-EKF:\n, it is %d -th loop',i);
    [FEKF_RMS.position(i, :), FEKF_RMS.orientation(i, :), FEKF_NEES.pose(i, :), FEKF_NEES.orientation(i, :)] = ...
        FEKF_plot_rms_nees( FEKF_result, data, 0 );
    
    
    RocEKF_result = RocEKF_SLAM( data );
        fprintf('Roc-EKF:\n, it is %d -th loop',i);
    [RocEKF_RMS.position(i, :), RocEKF_RMS.orientation(i, :), RocEKF_NEES.pose(i, :), RocEKF_NEES.orientation(i, :)] = ...
        RocEKF_plot_rms_nees( RocEKF_result, data, 0 );
    

    EKF_result = EKF_SLAM( data );
        fprintf('EKF:\n, it is %d -th loop',i);
    [EKF_RMS.position(i, :), EKF_RMS.orientation(i, :), EKF_NEES.pose(i, :), EKF_NEES.orientation(i, :)] = ...
        EKF_plot_rms_nees( EKF_result, data, 0 );
    toc;
    
end

[REKF_RMS_ave, REKF_NEES_ave] = compute_rms_nees_ave( REKF_RMS, REKF_NEES );
[FEKF_RMS_ave, FEKF_NEES_ave] = compute_rms_nees_ave( FEKF_RMS, FEKF_NEES );
[RocEKF_RMS_ave, RocEKF_NEES_ave] = compute_rms_nees_ave( RocEKF_RMS, RocEKF_NEES );
[EKF_RMS_ave, EKF_NEES_ave] = compute_rms_nees_ave( EKF_RMS, EKF_NEES );

figure;
%subplot(2,2,1);
plot(1:size(REKF_RMS_ave.position, 2), REKF_RMS_ave.position, 'r'); hold on;
plot(1:size(FEKF_RMS_ave.position, 2), FEKF_RMS_ave.position, 'g'); hold on;
plot(1:size(EKF_RMS_ave.position, 2), EKF_RMS_ave.position, 'k'); hold on;
plot(1:size(RocEKF_RMS_ave.position, 2), RocEKF_RMS_ave.position, 'b'); hold on;

legend('R-EKF', 'FEJ-EKF', 'EKF', 'Roc-EKF' ,'Location','northeast');
title('RMS:position(meter)');xlim([0,size(REKF_RMS_ave.position, 2)]);

figure;
plot(1:size(REKF_RMS_ave.orientation, 2), REKF_RMS_ave.orientation, 'r'); hold on;
plot(1:size(FEKF_RMS_ave.orientation, 2), FEKF_RMS_ave.orientation, 'g'); hold on;
plot(1:size(EKF_RMS_ave.orientation, 2), EKF_RMS_ave.orientation, 'k'); hold on;
plot(1:size(RocEKF_RMS_ave.orientation, 2), RocEKF_RMS_ave.orientation, 'b'); hold on;

legend('R-EKF', 'FEJ-EKF',  'EKF', 'Roc-EKF', 'Location','northeast');
title('RMS:orientation(rad)');xlim([0,size(REKF_RMS_ave.orientation, 2)]);

figure;
semilogy(1:size(REKF_NEES_ave.orientation, 2), REKF_NEES_ave.orientation, 'r'); hold on;
semilogy(1:size(FEKF_NEES_ave.orientation, 2), FEKF_NEES_ave.orientation, 'g'); hold on;
semilogy(1:size(EKF_NEES_ave.orientation, 2), EKF_NEES_ave.orientation, 'k'); hold on;
semilogy(1:size(RocEKF_NEES_ave.orientation, 2), RocEKF_NEES_ave.orientation, 'b'); hold on;

legend('R-EKF', 'FEJ-EKF', 'EKF', 'Roc-EKF', 'Location','northeast');
title('NEES:orientation');xlim([0,size(REKF_NEES_ave.orientation, 2)]);

figure;
% semilogy(1:size(REKF_NEES_ave.pose, 2), REKF_NEES_ave.pose, 'r'); hold on;
% semilogy(1:size(FEKF_NEES_ave.pose, 2), FEKF_NEES_ave.pose, 'g'); hold on;
% semilogy(1:size(EKF_NEES_ave.pose, 2), EKF_NEES_ave.pose, 'k'); hold on;
% semilogy(1:size(RocEKF_NEES_ave.pose, 2), RocEKF_NEES_ave.pose, 'b'); hold on;


plot(1:size(REKF_NEES_ave.pose, 2), REKF_NEES_ave.pose, 'r-'); hold on;
plot(1:size(FEKF_NEES_ave.pose, 2), FEKF_NEES_ave.pose, 'g+'); hold on;
plot(1:size(EKF_NEES_ave.pose, 2), EKF_NEES_ave.pose, 'ko'); hold on;
plot(1:size(RocEKF_NEES_ave.pose, 2), RocEKF_NEES_ave.pose, 'b'); hold on;
plot(1:size(RocEKF_NEES_ave.pose, 2), 1.12*ones( 1,size(RocEKF_NEES_ave.pose, 2)  ), 'r--'); hold on;
plot(1:size(RocEKF_NEES_ave.pose, 2), 0.89*ones( 1,size(RocEKF_NEES_ave.pose, 2)  ), 'r--'); hold on;
hleg=legend('R-EKF', 'FEJ-EKF', 'SO(3)-EKF', 'Robocentric-EKF', '95% confidence bound' , 'Location','northwest');
%hleg=legend('R-EKF', '95% confidence bound' , 'Location','northwest');
set(hleg,'FontSize',18)
ylabel('NEES of robot pose','fontsize',18);
xlabel('Time steps','fontsize',18);

xlim([0,size(REKF_NEES_ave.pose, 2)]);
title('NEES:pose');

fprintf('Mean values:\n');
fprintf('           RMS-Position    RMS-Orientation     NEES-Pose   NEES-Orientation\n');
fprintf('R-EKF:          %.5f            %.5f       %.5f           %.5f\n', ...
    mean(REKF_RMS_ave.position), mean(REKF_RMS_ave.orientation), mean(REKF_NEES_ave.pose), mean(REKF_NEES_ave.orientation));
fprintf('FEJ-EKF:          %.5f            %.5f       %.5f           %.5f\n', ...
    mean(FEKF_RMS_ave.position), mean(FEKF_RMS_ave.orientation), mean(FEKF_NEES_ave.pose), mean(FEKF_NEES_ave.orientation));
fprintf('EKF:            %.5f            %.5f      %.5f          %.5f\n', ...
    mean(EKF_RMS_ave.position), mean(EKF_RMS_ave.orientation), mean(EKF_NEES_ave.pose), mean(EKF_NEES_ave.orientation));
fprintf('Roc-EKF:            %.5f            %.5f      %.5f          %.5f\n', ...
    mean(RocEKF_RMS_ave.position), mean(RocEKF_RMS_ave.orientation), mean(RocEKF_NEES_ave.pose), mean(RocEKF_NEES_ave.orientation));


% comparison of following methods in 3D environment
% - left IEKF-SLAM 
% - right IEKF-SLAM
% - EKF-SLAM
% - ideal EKF-SLAM
clear;
clc;
close all;
% add directories
addpath('datagen_3d/');
addpath('left_iekf_3d/');
addpath('right_iekf_3d/');
addpath('lie_utils/');
addpath('ekf_3d/');
addpath('ideal_ekf_3d/');
% generate simulation data
niter = 50;
nsteps = 200;
RIEKF_RMS.position = [];
RIEKF_RMS.orientation = [];
RIEKF_NEES.pose = [];
RIEKF_NEES.orientation = [];

LIEKF_RMS.position = [];
LIEKF_RMS.orientation = [];
LIEKF_NEES.pose = [];
LIEKF_NEES.orientation = [];

Ideal_EKF_RMS.position = [];
Ideal_EKF_RMS.orientation = [];
Ideal_EKF_NEES.pose = [];
Ideal_EKF_NEES.orientation = [];

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
    
    % right iekf-slam
    RIEKF_result = RIEKF_SLAM( data );
    % RIEKF_plot_estimation( RIEKF_result, data );
    [RIEKF_RMS.position(i, :), RIEKF_RMS.orientation(i, :), RIEKF_NEES.pose(i, :), RIEKF_NEES.orientation(i, :)] = ...
        RIEKF_plot_rms_nees( RIEKF_result, data, 0 );
    
    % left iekf-slam
    LIEKF_result = LIEKF_SLAM( data );
    % LIEKF_plot_estimation( LIEKF_result, data );
    [LIEKF_RMS.position(i, :), LIEKF_RMS.orientation(i, :), LIEKF_NEES.pose(i, :), LIEKF_NEES.orientation(i, :)] = ...
        LIEKF_plot_rms_nees( LIEKF_result, data, 0 );
    
    % ideal ekf-slam
    Ideal_EKF_result = Ideal_EKF_SLAM( data );
    % Ideal_EKF_plot_estimation( Ideal_EKF_result, data );
    [Ideal_EKF_RMS.position(i, :), Ideal_EKF_RMS.orientation(i, :), Ideal_EKF_NEES.pose(i, :), Ideal_EKF_NEES.orientation(i, :)] = ...
        Ideal_EKF_plot_rms_nees( Ideal_EKF_result, data, 0 );
    
    % ekf-slam
    EKF_result = EKF_SLAM( data );
    % EKF_plot_estimation( EKF_result, data );
    [EKF_RMS.position(i, :), EKF_RMS.orientation(i, :), EKF_NEES.pose(i, :), EKF_NEES.orientation(i, :)] = ...
        EKF_plot_rms_nees( EKF_result, data, 0 );
    toc;
    
end

[RIEKF_RMS_ave, RIEKF_NEES_ave] = compute_rms_nees_ave( RIEKF_RMS, RIEKF_NEES );
[LIEKF_RMS_ave, LIEKF_NEES_ave] = compute_rms_nees_ave( LIEKF_RMS, LIEKF_NEES );
[Ideal_EKF_RMS_ave, Ideal_EKF_NEES_ave] = compute_rms_nees_ave( Ideal_EKF_RMS, Ideal_EKF_NEES );
[EKF_RMS_ave, EKF_NEES_ave] = compute_rms_nees_ave( EKF_RMS, EKF_NEES );

figure;
%subplot(2,2,1);
plot(1:size(RIEKF_RMS_ave.position, 2), RIEKF_RMS_ave.position, 'r'); hold on;
plot(1:size(LIEKF_RMS_ave.position, 2), LIEKF_RMS_ave.position, 'g'); hold on;
plot(1:size(Ideal_EKF_RMS_ave.position, 2), Ideal_EKF_RMS_ave.position, 'b'); hold on;
plot(1:size(EKF_RMS_ave.position, 2), EKF_RMS_ave.position, 'c'); hold on;
legend('RIEKF', 'LIEKF', 'Ideal EKF', 'EKF', 'Location','northeast');
title('RMS:position(meter)');xlim([0,size(RIEKF_RMS_ave.position, 2)]);

figure;
%subplot(2,2,2); 
plot(1:size(RIEKF_RMS_ave.orientation, 2), RIEKF_RMS_ave.orientation, 'r'); hold on;
plot(1:size(LIEKF_RMS_ave.orientation, 2), LIEKF_RMS_ave.orientation, 'g'); hold on;
plot(1:size(Ideal_EKF_RMS_ave.orientation, 2), Ideal_EKF_RMS_ave.orientation, 'b'); hold on;
plot(1:size(EKF_RMS_ave.orientation, 2), EKF_RMS_ave.orientation, 'c'); hold on;
legend('RIEKF', 'LIEKF', 'Ideal EKF', 'EKF', 'Location','northeast');
title('RMS:orientation(rad)');xlim([0,size(RIEKF_RMS_ave.orientation, 2)]);

figure;
%subplot(2,2,3);
semilogy(1:size(RIEKF_NEES_ave.orientation, 2), RIEKF_NEES_ave.orientation, 'r'); hold on;
semilogy(1:size(LIEKF_NEES_ave.orientation, 2), LIEKF_NEES_ave.orientation, 'g'); hold on;
semilogy(1:size(Ideal_EKF_NEES_ave.orientation, 2), Ideal_EKF_NEES_ave.orientation, 'b'); hold on;
semilogy(1:size(EKF_NEES_ave.orientation, 2), EKF_NEES_ave.orientation, 'c'); hold on;
legend('RIEKF', 'LIEKF', 'Ideal EKF', 'EKF', 'Location','northeast');
title('NEES:orientation');xlim([0,size(RIEKF_NEES_ave.orientation, 2)]);

figure;
%subplot(2,2,4);
semilogy(1:size(RIEKF_NEES_ave.pose, 2), RIEKF_NEES_ave.pose, 'r'); hold on;
semilogy(1:size(LIEKF_NEES_ave.pose, 2), LIEKF_NEES_ave.pose, 'g'); hold on;
semilogy(1:size(Ideal_EKF_NEES_ave.pose, 2), Ideal_EKF_NEES_ave.pose, 'b'); hold on;
semilogy(1:size(EKF_NEES_ave.pose, 2), EKF_NEES_ave.pose, 'c'); hold on;
legend('RIEKF', 'LIEKF', 'Ideal EKF', 'EKF', 'Location','northeast');
title('NEES:pose');xlim([0,size(RIEKF_NEES_ave.pose, 2)]);





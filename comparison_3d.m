% comparison of following methods in 3D environment
% - left IEKF-SLAM 
% - EKF-SLAM
% - ideal EKF-SLAM
clear;
clc;
close all;
% add directories
addpath('datagen_3d/');
addpath('left_iekf_3d/');
addpath('lie_utils/');
addpath('ekf_3d/');
addpath('ideal_ekf_3d/');
% generate simulation data
%data = gen_data();
load('data');
nposes = size(data.poses.position, 2);
nlandmarks = size(data.landmarks, 1);
fprintf('Generate %d poses and %d landmarks\n', nposes, nlandmarks);

% left iekf-slam
LIEKF_result = LIEKF_SLAM( data );
LIEKF_plot_estimation( LIEKF_result, data );
LIEKF_plot_rms_nees( LIEKF_result, data );

% ideal ekf-slam
Ideal_EKF_result = Ideal_EKF_SLAM( data );
Ideal_EKF_plot_estimation( Ideal_EKF_result, data );
Ideal_EKF_plot_rms_nees( Ideal_EKF_result, data );

% ekf-slam
EKF_result = EKF_SLAM( data );
EKF_plot_estimation( EKF_result, data );
EKF_plot_rms_nees( EKF_result, data );
% comparison of following methods in 3D environment
% - REKF SLAM 
% - FEJ-EKF SLAM
% - T-EKF-SLAM
clear;
clc;
close all;
% add directories
addpath('datagen_3d/');
addpath('f_ekf_3d/');

%addpath('Copy_of_f_ekf_3d/');

addpath('right_ekf_3d/');
addpath('lie_utils/');
addpath('ekf_3d/');
% generate simulation data
data = gen_data(0);
%load('data/1.mat');
nposes = size(data.poses.position, 2);
nlandmarks = size(data.landmarks, 1);
fprintf('Generate %d poses and %d landmarks\n', nposes, nlandmarks);

REKF_result = REKF_SLAM( data );
REKF_plot_estimation( REKF_result, data );
REKF_plot_rms_nees( REKF_result, data, 1 );

FEKF_result = FEKF_SLAM( data );
FEKF_plot_estimation( FEKF_result, data );
FEKF_plot_rms_nees( FEKF_result, data, 1 );

EKF_result = EKF_SLAM( data );
EKF_plot_estimation( EKF_result, data );
EKF_plot_rms_nees( EKF_result, data, 1 );

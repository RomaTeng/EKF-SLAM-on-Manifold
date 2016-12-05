% comparison of following methods in 3D environment
% - REKF SLAM 
% - FEJ-EKF SLAM
% - T-EKF-SLAM
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
data = gen_data(0);
%load('data/1.mat');
nposes = size(data.poses.position, 2);
nlandmarks = size(data.landmarks, 1);
fprintf('Generate %d poses and %d landmarks\n', nposes, nlandmarks);






 fprintf('R-EKF-mod\n');
REKF_result = REKF_SLAM( data );
%REKF_plot_estimation( REKF_result, data );
REKF_plot_rms_nees( REKF_result, data, 1 );




fprintf('F-EKF\n');
FEKF_result = FEKF_SLAM( data );
%FEKF_plot_estimation( FEKF_result, data );
FEKF_plot_rms_nees( FEKF_result, data, 1 );



fprintf('RobotCentric-EKF\n');
RocEKF_result = RocEKF_SLAM( data );
%RocEKF_plot_estimation( RocEKF_result, data );
RocEKF_plot_rms_nees( RocEKF_result, data, 1 );


fprintf('EKF\n');
EKF_result = EKF_SLAM( data );
%EKF_plot_estimation( EKF_result, data );
EKF_plot_rms_nees( EKF_result, data, 1 );




% fprintf('se3-EKF\n');
% se3EKF_result = se3EKF_SLAM( data );
% %REKF_plot_estimation( REKF_result, data );
% se3EKF_plot_rms_nees( se3EKF_result, data, 1 );


% fprintf('notR-EKF-EKF\n');
% notREKF_result = notREKF_SLAM( data );
% %REKF_plot_estimation( REKF_result, data );
% notREKF_plot_rms_nees( notREKF_result, data, 0 );


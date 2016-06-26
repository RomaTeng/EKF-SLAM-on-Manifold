function data = gen_data()
% generate data 

close all;
clear;
%addpath('lie_utils/');
config;

% step size
t = 0:1:300;
poses = gen_trajectory(t);

% plot 3d x y z
figure;
%scatter3( poses.position(1,:), poses.position(2, :), poses.position(3, :), 'r*' );
%hold on;
%draw axis
axisl = 5;
%{
for i = 1:size(poses.euler, 2)
    rotationi = poses.orientation((i-1)*3+1:i*3, :);
    xdir = poses.position(:, i)+axisl*rotationi(:, 1);
    xdir = [poses.position(:, i) xdir];
    ydir = poses.position(:, i)+axisl*rotationi(:, 2);
    ydir = [poses.position(:, i) ydir];
    zdir = poses.position(:, i)+axisl*rotationi(:, 3);
    zdir = [poses.position(:, i) zdir];
    plot3(xdir(1,:), xdir(2,:), xdir(3,:), 'Color', 'red'); hold on;
    plot3(ydir(1,:), ydir(2,:), ydir(3,:), 'Color', 'green'); hold on;
    plot3(zdir(1,:), zdir(2,:), zdir(3,:), 'Color', 'blue'); hold on;
end
%}
%% add random points
n_landmarks = N_LANDMARKS;
if ~exist('landmarks')
    % generate new landmarks
    border = [30, 30, 30];
    minpos = min(poses.position');
    maxpos = max(poses.position');
    minlm = minpos - border;
    maxlm = maxpos + border;
    landmarks(:, 1) = minlm(1)+rand(n_landmarks, 1)*(maxlm(1)-minlm(1));
    landmarks(:, 2) = minlm(2)+rand(n_landmarks, 1)*(maxlm(2)-minlm(2));
    landmarks(:, 3) = minlm(3)+rand(n_landmarks, 1)*(maxlm(3)-minlm(3));
    save('landmarks', 'landmarks');
else
    % load landmarks
    load('landmarks');
end

%% draw landmarks
scatter3( landmarks(:, 1), landmarks(:, 2), landmarks(:, 3), 'go' ); hold on;

%% generate odom
odoms = [];
for i = 1:size(poses.position, 2)-1;
    % compute odometry in so(3)
    rotationi = poses.orientation((i-1)*3+1:i*3, :);
    rotationi1 = poses.orientation(i*3+1:(i+1)*3, :);
    rotation_diff = rotationi'*rotationi1;
    rotation_diff_so3 = so3_log(rotation_diff);
    translation_diff = poses.position(:, i+1)-poses.position(:, i);
    translation_diff = rotationi'*translation_diff;        % change
    if ADD_NOISE == 1
       noise_rand=randn(6,1);
        while(max(abs(noise_rand))> 1.5)
           noise_rand=randn(6,1);
        end
       noise_odo=ODOM_NOISE^(1/2)*noise_rand;
       rotation_diff_so3=rotation_diff_so3+noise_odo(4:6);
       translation_diff=translation_diff+noise_odo(1:3);
    end
   
    odomi = [translation_diff; rotation_diff_so3]';
    odoms = [odoms; odomi];
end

%% generate observations
obsers = {};

for i = 1:size(poses.position, 2) % for pose
    posi = poses.position(:, i);
    rotationi = poses.orientation((i-1)*3+1:i*3, :);
    
    xdir = poses.position(:, i)+axisl*rotationi(:, 1);
    xdir = [poses.position(:, i) xdir];
    ydir = poses.position(:, i)+axisl*rotationi(:, 2);
    ydir = [poses.position(:, i) ydir];
    zdir = poses.position(:, i)+axisl*rotationi(:, 3);
    zdir = [poses.position(:, i) zdir];
    plot3(xdir(1,:), xdir(2,:), xdir(3,:), 'Color', 'red'); hold on;
    plot3(ydir(1,:), ydir(2,:), ydir(3,:), 'Color', 'green'); hold on;
    plot3(zdir(1,:), zdir(2,:), zdir(3,:), 'Color', 'blue'); hold on;
    
    zi = rotationi(:, 3)';
    obseri = [];
    for j = 1:size(landmarks, 1)
        ptjdir = landmarks(j, :) - posi';
        ptjangle = dot(ptjdir, zi)/(norm(ptjdir)*norm(zi));
        ptjangle = acos(ptjangle);
        if ptjangle < MAX_DEGREE && norm(ptjdir) < MAX_RANGE
            % scatter3( landmarks(j, 1), landmarks(j, 2), landmarks(j, 3), 'r*' );
            % hold on;
            ptjdir = rotationi'*ptjdir';
            obserij = [j, ptjdir'];
            if 0 %ADD_NOISE == 1
                noise_rand=randn(3,1);
             while(max(abs(noise_rand))> 1.5)
                 noise_rand=randn(3,1);
             end
                noise_ob=OBSV_NOISE^(1/2)*noise_rand;
                obserij(2:4)=obserij(2:4)+noise_ob';  %change
            end
            obseri = [obseri; obserij];
        end
    end
    obsers{i} = obseri;
end
title('3D Simulation Data Generator');


%% convert data format
indy = 1;
for i = 1:size(odoms, 1)
    % set observation
    for j = 1:size(obsers{i}, 1)
        ztfile(indy, 1) = obsers{i}(j, 2); 
        ztfile(indy, 2) = 2;
        ztfile(indy, 3) = obsers{i}(j, 1);
        ztfile(indy, 4) = i-1;
        indy = indy+1;
        ztfile(indy, 1) = obsers{i}(j, 3); 
        ztfile(indy, 2) = 2;
        ztfile(indy, 3) = obsers{i}(j, 1);
        ztfile(indy, 4) = i-1;
        indy = indy+1;
        ztfile(indy, 1) = obsers{i}(j, 4); 
        ztfile(indy, 2) = 2;
        ztfile(indy, 3) = obsers{i}(j, 1);
        ztfile(indy, 4) = i-1;
        indy = indy+1;
    end
    % set odometry
    for j = 1:6
        ztfile(indy, 1) = odoms(i, j); 
        ztfile(indy, 2) = 1;
        ztfile(indy, 3) = i;
        ztfile(indy, 4) = i-1;
        indy = indy+1;
    end  
end

% set observation for the last frame
index = length(obsers);
for j = 1:size(obsers{index}, 1)
    ztfile(indy, 1) = obsers{index}(j, 2); 
    ztfile(indy, 2) = 2;
    ztfile(indy, 3) = obsers{index}(j, 1);
    ztfile(indy, 4) = index-1;
    indy = indy+1;
    ztfile(indy, 1) = obsers{index}(j, 3); 
    ztfile(indy, 2) = 2;
    ztfile(indy, 3) = obsers{index}(j, 1);
    ztfile(indy, 4) = index-1;
    indy = indy+1;
    ztfile(indy, 1) = obsers{index}(j, 4); 
    ztfile(indy, 2) = 2;
    ztfile(indy, 3) = obsers{index}(j, 1);
    ztfile(indy, 4) = index-1;
    indy = indy+1;
end
data.state = ztfile; % state vector
data.obse_cov = OBSV_NOISE; % observation covariance matrix
data.odom_cov = ODOM_NOISE; % odometry covariance matrix

data.odom_sigma = SIGMA_ODOM;
data.obsv_sigma = SIGMA_OBSV;

data.landmarks=landmarks;
data.poses=poses;
save data data

clearvars -except data;


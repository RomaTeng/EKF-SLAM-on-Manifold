function [] = EKF_plot_rms_nees( estimation_results, data )
% plot rms and nees for left-invariant ekf
figure;
N  = size(estimation_results, 2);

T = 1:N;
RMS_position=[];
RMS_orientation=[];
for i = T
    position = estimation_results{i}.position;   
    ap = data.poses.position(:,i);
    
    RMS_position = [RMS_position norm(position-ap)];
    RMS_orientation=  [RMS_orientation norm(so3_log((data.poses.orientation(3*i-2:3*i,1:3))'*estimation_results{i}.orientation))];
       
end
fprintf( 'mean(RMS:Position) = %f\n', mean(RMS_position) );
fprintf( 'mean(RMS:Orientation) = %f\n', mean(RMS_orientation) );


subplot(2,2,1);plot(T, RMS_position);
title('RMS:position(meter)');xlim([0,N]);
subplot(2,2,2); plot(T,RMS_orientation);
title('RMS:orientation(radius)');xlim([0,N]);


 NEES_pose=[];
 NEES_orientation=[];
for i = T
    position = estimation_results{i}.position;   
    ap = data.poses.position(:,i);
    
    dw=so3_log((data.poses.orientation(3*i-2:3*i,1:3))'*estimation_results{i}.orientation);
    dv=( (estimation_results{i}.position-data.poses.position(:,i)));
    
%     inFormation=inv(EstimationHistory{i}.cov);
%     cov_o=inFormation(1:3,1:3);
%     cov_pose=inFormation(1:6,1:6);
     cov_o=estimation_results{i}.cov(1:3,1:3);
     cov_pose=estimation_results{i}.cov(1:6,1:6);
%      cov_pose(1:3,4:6)=zeros(3,3);
%      cov_pose(4:6,1:3)=zeros(3,3);
     invcov_o=eye(3)/cov_o;
     invcov_pose=eye(6)/cov_pose;
    dP=[dw;dv];
    
    NEES_orientation = [NEES_orientation dw'*invcov_o*dw/3];
    NEES_pose=  [NEES_pose dP'*invcov_pose*dP/6];
       
end
fprintf( 'mean(NEES:Position) = %f\n', mean(NEES_pose(2:end)) );
fprintf( 'mean(NEES:Orientation) = %f\n', mean(NEES_orientation(2:end)) );

subplot(2,2,3);
plot(T,NEES_orientation);
title('NEES:orientation');xlim([0,N]);
subplot(2,2,4);
plot(T,NEES_pose);
title('NEES:pose');;xlim([0,N]);



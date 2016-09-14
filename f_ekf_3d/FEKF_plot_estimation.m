function [] = FEKF_plot_estimation( estimation_result, data )



%% plot ground truth poses and estimated poses
errposition = [];
N  = size(estimation_result, 2);
figure;
title('Lie Group EKF: Left')
T = 1:N;
for i = T
    position = estimation_result{i}.position;
    ap = data.poses.position(:,i);
    
end

%% plot error in position


N  = size(estimation_result, 2);
NL = size(estimation_result{N}.landmarks, 2);

ErrLandmark = zeros(1, NL);
for j = 1:NL
    ErrLandmark(j) = norm(estimation_result{N}.landmarks(1:3,j)-data.landmarks( estimation_result{N}.landmarks(4,j),1:3)');
end
axis equal;

%% plot error in orientation
dT = 1:N;
ThetaVector=zeros(3,N);
SigmaTheta=zeros(3,N);
for j = 1:N
    ThetaVector(:,j)=so3_log((data.poses.orientation(3*j-2:3*j,1:3))'*estimation_result{j}.orientation);
    SigmaTheta(:,j)= [sqrt(estimation_result{j}.cov(1,1));sqrt(estimation_result{j}.cov(2,2));sqrt(estimation_result{j}.cov(3,3))];
end

subplot(3,2,1)
plot(dT,ThetaVector(1,:),'g'); hold on;
plot(dT,3*SigmaTheta(1,:),'r'); hold on;
plot(dT,-3*SigmaTheta(1,:),'r'); hold on;
xlim([0,N]);
title('3\sigma bound: \alpha');

subplot(3,2,3)
plot(dT,ThetaVector(2,:),'g'); hold on;
plot(dT,3*SigmaTheta(2,:),'r'); hold on;
plot(dT,-3*SigmaTheta(2,:),'r'); hold on;
xlim([0,N]);
title('3\sigma bound: \beta');

subplot(3,2,5)
plot(dT,ThetaVector(3,:),'g'); hold on;
plot(dT,3*SigmaTheta(3,:),'r'); hold on;
plot(dT,-3*SigmaTheta(3,:),'r'); hold on;
xlim([0,N]);
title('3\sigma bound: \gamma');

% figure;
PositionVector=zeros(3,N);
Sigma=zeros(3,N);
for j = 1:N
    PositionVector(:,j)=inv(jaco_r(-ThetaVector(:,j)))*( data.poses.orientation(3*j-2:3*j,1:3)' *(estimation_result{j}.position-data.poses.position(:,j)));
    Sigma(:,j)= [sqrt(estimation_result{j}.cov(4,4));sqrt(estimation_result{j}.cov(5,5));sqrt(estimation_result{j}.cov(6,6))];
end
subplot(3,2,2)
plot(dT,PositionVector(1,:),'g'); hold on;
plot(dT,3*Sigma(1,:),'r'); hold on;
plot(dT,-3*Sigma(1,:),'r'); hold on;
xlim([0,N]);
title('3\sigma bound: x');


subplot(3,2,4)
plot(dT,PositionVector(2,:),'g'); hold on;
plot(dT,3*Sigma(2,:),'r'); hold on;
plot(dT,-3*Sigma(2,:),'r'); hold on;
xlim([0,N]);
title('3\sigma bound: y');


subplot(3,2,6)
plot(dT,PositionVector(3,:),'g'); hold on;
plot(dT,3*Sigma(3,:),'r'); hold on;
plot(dT,-3*Sigma(3,:),'r'); hold on;
xlim([0,N]);
title('3\sigma bound: z');

ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1], ...
          'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');

text(0.5, 1,'\bf Lie Group EKF: Left','HorizontalAlignment' ,'center','VerticalAlignment', 'top');

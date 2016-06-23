% RMS

N  = size(EstimationHistory, 2);

T = 1:N;
RMS_position=[];
RMS_orientation=[];
for i = T
    position = EstimationHistory{i}.position;   
    ap = data.poses.position(:,i);
    
    RMS_position = [RMS_position norm(position-ap)];
    RMS_orientation=  [RMS_orientation norm(so3_log(EstimationHistory{i}.orientation*(data.poses.orientation(3*i-2:3*i,1:3))'))];
       
end

subplot(2,2,1)
plot(T,RMS_position)
title('RMS:position, m')
subplot(2,2,2)
plot(T,RMS_orientation)
title('RMS:orientation, rad')

 
for i = T
    position = EstimationHistory{i}.position;   
    ap = data.poses.position(:,i);
    
    dw=so3_log(EstimationHistory{i}.orientation*(data.poses.orientation(3*i-2:3*i,1:3))');
    dv=
    
    
    
    
    NEES_pose = [NEES_pose a];
    NEES_orientation=  [NEES_orientation b];
       
end
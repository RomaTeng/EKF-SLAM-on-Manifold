%%%%% Generate "CameraPose.txt" and "CameraMeasurement.txt"  for GTSAM test

load data.mat;

fp=fopen('CameraPose.txt','w');
NumberOfPoses=size(data.poses.position,2);                % From 0 to end
for i=0:NumberOfPoses-1
    Index=i;
    Orientation=data.poses.orientation(3*i+1:3*i+3,1:3);
    Position=data.poses.position(1:3,i+1);
    
    Pose=[Orientation Position;zeros(1,3) 1]';
    
fprintf(fp,'%d ',Index);
for j=1:16
fprintf(fp,'%16.8f ',Pose(j));
end
fprintf(fp,'\n');

end
fclose(fp);



fp=fopen('CameraMeasurement.txt','w');
Number=size(data.state,1)/3;               
for i=1:Number
    
    DataMatrix=data.state(3*i-2:3*i,:);
    flag=DataMatrix(1,2);
    if flag==2
    Landmark_Local=DataMatrix(1:3,1);
    Landmark_ID=DataMatrix(1,3);
    Pose_ID=DataMatrix(1,4);
    
    Orientation=data.poses.orientation(3*Pose_ID+1:3*Pose_ID+3,1:3);
    Position=data.poses.position(1:3,Pose_ID+1);    
    Pose=[Orientation Position;zeros(1,3) 1];
        
    Landmark=data.landmarks(Landmark_ID,1:3)';
    y=StereoProjection(Pose,Landmark);  % y=[ur ul v x_local y_local z_local];
    
    fprintf(fp,'%d ',Pose_ID);
    fprintf(fp,'%d ',Landmark_ID);

    for j=1:6
     fprintf(fp,'%16.8f ',y(j));
    end
     fprintf(fp,'\n');
    
    
    
    end
end
fclose(fp);
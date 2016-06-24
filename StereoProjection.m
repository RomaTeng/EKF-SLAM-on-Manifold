function y=StereoProjection(Pose,landmark)

R=Pose(1:3,1:3);
P=Pose(1:3,4);

Local=R'*(landmark-P);

K=[];

ur=12;
ul=11;
v=3;




y=[Local' Local'];


end
function [ X ] = SpecialAdd( X,S )

s_theta=S(1:3);
s_p=S(4:6);

sizeS=size(S,1);
NumberOfLandmarks=(sizeS-6)/3;


X.position=X.position+s_p;



if NumberOfLandmarks>=1
s_landmarksMatrix=reshape(S(7:end),3,NumberOfLandmarks);
X.landmarks(1:3,:)=X.landmarks(1:3,:)+s_landmarksMatrix;
end

X.orientation=Exp(s_theta)*X.orientation;


end




function [ X ] = special_add_left( X, S )
% add on Lie group-Lie Algebra space
    s_theta=S(1:3);
    s_p=S(4:6);
    sizeS=size(S,1);
    NumberOfLandmarks=(sizeS-6)/3;
    X.position=X.position+X.orientation*jaco_r(-s_theta)*s_p;
    if NumberOfLandmarks>=1
        s_landmarksMatrix=reshape(S(7:end),3,NumberOfLandmarks);
        X.landmarks(1:3,:)=X.landmarks(1:3,:)+X.orientation*jaco_r(-s_theta)*s_landmarksMatrix;
    end
    X.orientation=X.orientation*so3_exp(s_theta);
end




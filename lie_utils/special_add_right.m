function [ X ] = special_add_right( X,S )

    s_theta=S(1:3);
    s_p=S(4:6);

    sizeS=size(S,1);
    NumberOfLandmarks=(sizeS-6)/3;

    Exps=so3_exp(s_theta);

    X.position= Exps*X.position+jaco_r(-s_theta)*s_p;



    if NumberOfLandmarks>=1
        s_landmarksMatrix=reshape(S(7:end),3,NumberOfLandmarks);
        X.landmarks(1:3,:)=Exps*X.landmarks(1:3,:)+jaco_r(-s_theta)*s_landmarksMatrix;
    end

    X.orientation=Exps*X.orientation;


end
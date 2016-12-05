function [Estimation_X] = RocEKF_propagate(Estimation_X, OdometryFromThis2Next, Sigma_ODO )



% retrieve v and w
v = OdometryFromThis2Next(1:3);
w = OdometryFromThis2Next(4:6);



NumberOfLandmarks = size(Estimation_X.landmarks, 2);
Jrw = jaco_r(-w);
ExpMinusM = so3_exp(-w);




temp = repmat({Estimation_X.orientation}, 2,1 );
adRobot = blkdiag(temp{:});
 adRobot(4:6,1:3)=skew(Estimation_X.position)*Estimation_X.orientation;
 
B2=zeros(3*NumberOfLandmarks ,  6 ); 
if NumberOfLandmarks>0
    for i=1:NumberOfLandmarks
    %B2( 3*i-2:3*i  , :  )= ExpMinusM*[  skew( Estimation_X.landmarks(1:3,i) -v  )*Jrw , -eye(3) ];
    B2( 3*i-2:3*i  , :  )= ExpMinusM*[  skew( Estimation_X.landmarks(1:3,i)   ) , -eye(3) ];
    %B2=B2*[eye(3) zeros(3,3); -skew(v) eye(3) ];
    end
end


%B1=-[Jrw  zeros(3,3); skew(v)*Jrw eye(3)];
B1=[eye(3)  zeros(3,3); zeros(3,3) eye(3)];
adA= [  adRobot* B1;  B2    ];
%adA= [  -adRobot* B1; -B2    ];

odoCov=diag([w.^2;v.^2])*Sigma_ODO^2;



temp = repmat({ ExpMinusM  }, 2+NumberOfLandmarks,1 );
AA = blkdiag(temp{:});
AA(1:6,1:6)=eye(6);




% final update the covariance
Estimation_X.cov = AA*Estimation_X.cov*AA'+ adA*odoCov*adA';

% update position and orientation
Estimation_X.position = Estimation_X.position+Estimation_X.orientation*v;
Estimation_X.orientation = Estimation_X.orientation*so3_exp(w);

% update the local coordinates of landmarks
if NumberOfLandmarks>0

landmarks=Estimation_X.landmarks(1:3,:);
vv=repmat(v, 1, NumberOfLandmarks);
landmarks=ExpMinusM*(landmarks - vv  );
Estimation_X.landmarks(1:3,:)=landmarks; 

end

end





function [Estimation_X] = EKF_propagate(Estimation_X, OdometryFromThis2Next, odom_sigma )

v = OdometryFromThis2Next(1:3);
w = OdometryFromThis2Next(4:6);

% update position and orientation
Estimation_X.position = Estimation_X.position+Estimation_X.orientation*v;
orientation=Estimation_X.orientation;
Estimation_X.orientation = Estimation_X.orientation*Exp(w);


NumberOfLandmarks = size(Estimation_X.landmarks, 2);
%Jrw = J_r(-w);
%ExpMinusM = Exp(-w);


%G1 = [-orientation*Jrw zeros(3,3);zeros(3,3)  -orientation];
%G1 = [-orientation zeros(3,3);zeros(3,3)  -orientation];
G1=[orientation zeros(3,3); -skew(orientation*v)   orientation];

G= [G1; zeros(3* NumberOfLandmarks ,6)];
odoCov=diag([w.^2;v.^2])*odom_sigma^2;
W = G*odoCov*G';



% compute matrix A_{n}
temp = repmat({ eye(3) }, NumberOfLandmarks+2,1 );
A = blkdiag(temp{:});
A(1:3,1:3) = eye(3);% ExpMinusM;
A(4:6,1:3) = -skew(orientation*v);


% final update the covariance
Estimation_X.cov = A*Estimation_X.cov*A'+W;



end





function [Estimation_X] = Ideal_EKF_propagate(Estimation_X, OdometryFromThis2Next, odom_sigma, Aorientation, Aposition )
% function [Estimation_X] = LEKFonestepPropagate(Estimation_X, OdometryFromThis2Next, odoCov )
% Estimation_X      - current state and everything
% OdometryFromThis2Next
%                   - odometry 
% odoCov            - odometry covariance

% retrieve v and w
v = OdometryFromThis2Next(1:3);
w = OdometryFromThis2Next(4:6);

% update position and orientation
Estimation_X.position = Estimation_X.position+Estimation_X.orientation*v;
orientation=Estimation_X.orientation;
Estimation_X.orientation = Estimation_X.orientation*so3_exp(w);


NumberOfLandmarks = size(Estimation_X.landmarks, 2);
Jrw = jaco_r(w);
ExpMinusM = so3_exp(-w);


W1 = [-Jrw zeros(3,3);zeros(3,3)  Aorientation];
odoCov=diag([w.^2;v.^2])*odom_sigma^2;
W1 = W1*odoCov*W1';

W = sparse(3*NumberOfLandmarks+6,3*NumberOfLandmarks+6);
W(1:6,1:6) = W1;

% compute matrix A_{n}
temp = repmat({ eye(3) }, NumberOfLandmarks+2,1 );
A = blkdiag(temp{:});
A(1:3,1:3) = ExpMinusM;
A(4:6,1:3) = Aorientation*skew(v);


% final update the covariance
Estimation_X.cov = A*Estimation_X.cov*A'+W;

end





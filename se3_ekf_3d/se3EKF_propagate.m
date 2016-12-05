function [Estimation_X] = se3EKF_propagate(Estimation_X, OdometryFromThis2Next, Sigma_ODO )



% retrieve v and w
v = OdometryFromThis2Next(1:3);
w = OdometryFromThis2Next(4:6);



NumberOfLandmarks = size(Estimation_X.landmarks, 2);
Jrw = jaco_r(-w);
ExpMinusM = so3_exp(-w);



temp = repmat({Estimation_X.orientation}, 2,1 );
adRobot = blkdiag(temp{:});
 adRobot(4:6,1:3)=skew(Estimation_X.position)*Estimation_X.orientation;


B=-[-Jrw  zeros(3,3); -skew(v)*Jrw -eye(3)];


adA= -adRobot*B  ;


odoCov=diag([w.^2;v.^2])*Sigma_ODO^2;

% final update the covariance
Estimation_X.cov(1:6,1:6) = Estimation_X.cov(1:6,1:6)+ adA*odoCov*adA';

% update position and orientation
Estimation_X.position = Estimation_X.position+Estimation_X.orientation*v;
Estimation_X.orientation = Estimation_X.orientation*so3_exp(w);


end





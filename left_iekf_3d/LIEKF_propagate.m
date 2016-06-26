function [Estimation_X] = LIEKF_propagate(Estimation_X, OdometryFromThis2Next, odom_sigma )
% function [Estimation_X] = LEKFonestepPropagate(Estimation_X, OdometryFromThis2Next, odoCov )
% Estimation_X      - current state and everything
% OdometryFromThis2Next
%                   - odometry 
% odoCov            - odometry covariance

    % retrieve v and w
    v = OdometryFromThis2Next(1:3);
    w = OdometryFromThis2Next(4:6);

    % compute odometry covariance matrix
    odoCov=diag([w.^2;v.^2])*odom_sigma^2;
    
    % update position and orientation
    Estimation_X.position = Estimation_X.position+Estimation_X.orientation*v;
    Estimation_X.orientation = Estimation_X.orientation*so3_exp(w);


    NumberOfLandmarks = size(Estimation_X.landmarks, 2);
    Jrw = jaco_r(w);
    ExpMinusM = so3_exp(-w);


    W1 = [-Jrw zeros(3,3);zeros(3,3)  -ExpMinusM];
    W1 = W1*odoCov*W1';

    W = sparse(3*NumberOfLandmarks+6,3*NumberOfLandmarks+6);
    W(1:6,1:6) = W1;

    % compute matrix A_{n}
    temp = repmat({ExpMinusM}, NumberOfLandmarks+2,1 );
    A = blkdiag(temp{:});
    A(4:6,1:3) = -ExpMinusM*skew(v);


    % final update the covariance
    Estimation_X.cov = A*Estimation_X.cov*A'+W;

end





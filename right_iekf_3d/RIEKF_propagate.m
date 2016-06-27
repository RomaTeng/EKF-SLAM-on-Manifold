function [Estimation_X] = RIEKF_propagate(Estimation_X, OdometryFromThis2Next, Sigma_ODO )
% function [Estimation_X] = LEKFonestepPropagate(Estimation_X, OdometryFromThis2Next, odoCov )
% Estimation_X      - current state and everything
% OdometryFromThis2Next
%                   - odometry 
% odoCov            - odometry covariance

% retrieve v and w
v = OdometryFromThis2Next(1:3);
w = OdometryFromThis2Next(4:6);



NumberOfLandmarks = size(Estimation_X.landmarks, 2);
Jrw = jaco_r(-w);
ExpMinusM = so3_exp(-w);




temp = repmat({Estimation_X.orientation}, NumberOfLandmarks+2,1 );
A = blkdiag(temp{:});
 A(4:6,1:3)=skew(Estimation_X.position)*Estimation_X.orientation;
 
if NumberOfLandmarks>0
   for i=1:NumberOfLandmarks
    A(6+3*i-2:6+3*i,1:3)=skew(Estimation_X.landmarks(1:3,i))*Estimation_X.orientation;
   end
end

%A=sparse(A);

B1=[-Jrw  zeros(3,3); -skew(v)*Jrw -eye(3)];
B=[B1; sparse(3*NumberOfLandmarks,6)];
B=sparse(B);

adA=A*B;

odoCov=diag([w.^2;v.^2])*Sigma_ODO^2;
%odoCov=diag([odoCov(4,4),odoCov(5,5),odoCov(6,6),odoCov(1,1),odoCov(2,2),odoCov(3,3)]);

% final update the covariance
Estimation_X.cov = Estimation_X.cov+ adA*odoCov*adA';

% update position and orientation
Estimation_X.position = Estimation_X.position+Estimation_X.orientation*v;
Estimation_X.orientation = Estimation_X.orientation*so3_exp(w);


end





function [Estimation_X, FirstPosition] = FEKF_propagate(Estimation_X, OdometryFromThis2Next, Sigma_ODO , FirstPosition )
v = OdometryFromThis2Next(1:3);
w = OdometryFromThis2Next(4:6);

% update position and orientation
Estimation_X.position = Estimation_X.position+Estimation_X.orientation*v;
orientation=Estimation_X.orientation;
Estimation_X.orientation = Estimation_X.orientation*Exp(w);


NumberOfLandmarks = size(Estimation_X.landmarks, 2);
Jrw = J_r(-w);


%G1 = [-orientation*Jrw zeros(3,3);zeros(3,3)  -orientation];
%G1 = [-orientation zeros(3,3);zeros(3,3)  -orientation];
G1 = [orientation zeros(3,3);-skew(orientation*v)  orientation];

G= [G1; zeros(3* NumberOfLandmarks ,6)];
odoCov=diag([w.^2;v.^2])*Sigma_ODO^2;
W = G*odoCov*G';



% compute matrix A_{n}
temp = repmat({ eye(3) }, NumberOfLandmarks+2,1 );
A = blkdiag(temp{:});
A(1:3,1:3) = eye(3);% ExpMinusM;

if isempty(FirstPosition)
   A(4:6,1:3) = -skew(orientation*v);
else
   A(4:6,1:3) = -skew(Estimation_X.position-FirstPosition); 
end



% final update the covariance
Estimation_X.cov = A*Estimation_X.cov*A'+W;


FirstPosition=Estimation_X.position;
end





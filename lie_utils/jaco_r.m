function result = jaco_r( delta )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function: Overloading function.
%   Method1:   I(3x3) - (1-cos(theta))*skew(delta)/norm(delta)^2 + ...
%             (norm(delta)-sin(norm(delta)))*skew(delta)^2/norm(delta)^3
%   Input1:    delta: 3x1 vector
%   Returns1:  result:     See the document

%   Method2:   See the document
%   Input2:    delta: 6+3N vector
%   Returns2:  result:     See the document

%   Author:   Jingwei Song.   04/06/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N= (size(delta,1)-3)/3;        

if(size(delta,1) == 3)
    function_mode = 1;
else
    function_mode = 2;
end

if(function_mode == 1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function1
    theta = norm(delta);
    if(theta < 0.00000001)
        result = eye(3,3);
    else
        result = eye(3,3) - (1-cos(theta))*skew(delta)/theta^2 + (theta-sin(theta))*skew(delta)^2/norm(delta)^3;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
else
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function2
    result  = zeros(3+3*N,3+3*N);
    s_theta = delta(1:3,1);
    for i = 1 : N+1           % Set diagonal to J_r(s_theta)
        result(3*i-2:3*i,3*i-2:3*i) = jaco_r(s_theta);
    end
    for i = 1 : N
        s_temp = delta(3*i+1:3*i+3,1);
        result(3*i+1:3*i+3,1:3) = K_r_x1x2(s_theta, s_temp);
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
end


end


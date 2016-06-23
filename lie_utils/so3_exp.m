function [ result ] = so3_exp( x )
% compute the exponential mapping of R^3 to SO(3)
% use sparse matrix
	N = size(x,1);
	%if N == 3
        theta=norm(x);
        if theta==0
            result=eye(3);
        else     
            omega =x/theta;
            result=eye(3,3) + sin(theta) * skew(omega) + (1 - cos(theta))*skew(omega)^2;  
        end
		      
    %else
        % todo, compute exp() of R^6 to SE(3)?
    %    m = (N-3)/3;
    %    result=speye(3+m);
    %    result(1:3,1:3)=Exp( x(1:3));
    %end
end


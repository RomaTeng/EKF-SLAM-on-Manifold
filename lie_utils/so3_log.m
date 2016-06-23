function f = so3_log(R, varargin)

if (nargin>1)
    if (norm(R-eye(3),'fro') < 2*eps)
        f = zeros(3,1);
        return
    end
end

phi = acos(1/2*(trace(R)-1));

if (nargin>1)
    if (abs(phi) < 1e-10)
        f = zeros(3,1);
        return
    end
end

if norm(R-eye(3))>0.00001
    f = so3_hatinv(phi/(2*sin(phi))*(R-R'));
else
    f=[0;0;0];
end
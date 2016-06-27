function y = euler2rotation_matrix( X )
% compute rotation matrix from euler angle
% X     - euler angle
% y     - rotation matrix
    a = X(1); %x
    b = X(2); %y
    c = X(3); %z
    y = [cos(c)*cos(b) -sin(c)*cos(a)+cos(c)*sin(b)*sin(a) sin(c)*sin(a)+cos(c)*cos(a)*sin(b);...
         sin(c)*cos(b)  cos(c)*cos(a)+sin(a)*sin(b)*sin(c) -cos(c)*sin(a)+sin(b)*sin(c)*cos(a);...
         -sin(b) cos(b)*sin(a) cos(b)*cos(a)];
end

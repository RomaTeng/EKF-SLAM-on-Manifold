function [ Vehicle ] = gen_trajectory( t )
% generate trajectory given step t
% t     - step
    radius = 5;   
    % generate x y z
    Vehicle.position = radius* [ 5*cos(0.3*t); 4*sin(0.2*t); 2*sin(0.2*t+1) ];
    Vehicle.euler = [0.5*t+2; -0.3*t; 0.4*t];
    % generate rotation matrix
    for i = 1:size(Vehicle.euler, 2)
        Vehicle.orientation((i-1)*3+1:i*3, 1:3) = euler2rotation_matrix(Vehicle.euler(:, i));
    end
end

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

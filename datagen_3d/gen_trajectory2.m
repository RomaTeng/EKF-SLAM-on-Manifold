function [ Vehicle ] = gen_trajectory2( t )
% generate trajectory given step t
% t     - step
     radius = 10;   
%     % generate x y z
%     Vehicle.position = radius* [ 5*cos(0.3*t); 4*sin(0.2*t); 2*sin(0.2*t+1) ];
%     Vehicle.euler = [0.5*t+2; -0.3*t; 0.4*t];
    
    
        Vehicle.position = radius* [ 5.1*cos(0*t); 4.03*sin(0*t); 2.1*sin(0*t+1) ];
    Vehicle.euler =  [0*t+3; -0*t+2; 0*t+1];

    
    
    
    
    % generate rotation matrix
    for i = 1:size(Vehicle.euler, 2)
        Vehicle.orientation((i-1)*3+1:i*3, 1:3) = euler2rotation_matrix(Vehicle.euler(:, i));
    end
end


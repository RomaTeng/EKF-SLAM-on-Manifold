function y = skew( x )
% compute the skew symetric matrix 
% given input 3x1 vector x
    y=[0 -x(3) x(2);...
       x(3) 0 -x(1);...
       -x(2) x(1) 0];
end


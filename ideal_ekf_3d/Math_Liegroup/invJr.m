function y=invJr(x)
nx=norm(x);

y=eye(3)+1/2*skew(x)+(1/nx^2-(1+cos(nx))/(2*nx*sin(nx)))*skew(x)^2;



end
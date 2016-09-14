function [ result ] = Exp( x )

% use sparse matrix
N=size(x,1);

if N==3
    theta=norm(x);
    if theta==0
        result=eye(3);
    else     
        omega =x/theta;
        result=eye(3,3) + sin(theta) * skew(omega) + (1 - cos(theta))*skew(omega)^2;  
    end
        
else
    m=(N-3)/3;
    result=speye(3+m);
    result(1:3,1:3)=Exp( x(1:3));
    
    

end




    
    
    
end


function [ Jrs ] = JJJr( S )

if size(S,1)==3
 Jrs=J_r(S);
else
N=(size(S,1)-3)/3;
temp=repmat({J_r(S(1:3))}, N+1,1 );
      Jrs=blkdiag(temp{:});
 
      for i=1:N
      
      Jrs(3*i+1:3*i+3,1:3)=K_r_x1x2( S(1:3), S(3*i+1:3*i+3) );    
    
      end
    
end

end


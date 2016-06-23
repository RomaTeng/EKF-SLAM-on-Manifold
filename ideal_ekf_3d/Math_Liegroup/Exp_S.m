function [ y ] = Exp_S( S )

N=(size(S,1)-3)/3;


R=Exp(S(1:3));
JMr=J_r(-S(1:3));

p=reshape( S(4:end),3,N );

P=JMr*p;

y=eye(3+N);

y(1:3,:)=[R P];




end


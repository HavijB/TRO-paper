function g = expvector(xi,q)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
omega=xi(1:3);
v=xi(4:6);
%R=expm(skewsym(omega*q));
R=eye(3)+skewsym(omega)*sin(q)+(skewsym(omega))^2*(1-cos(q));
g=[R (eye(3)-R)*(skewsym(omega)*v)+omega*transpose(omega)*v*q; 0 0 0 1];
end
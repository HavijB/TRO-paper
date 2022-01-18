function [Ad] = Adjoint(R,p)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
p_tilde=skewsym(p);
Ad=[R p_tilde*R;zeros(3) R];
end
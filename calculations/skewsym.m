function [A_tilde] = skewsym(A)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
A_tilde=[0      -A(3)   A(2);
         A(3)   0       -A(1);
         -A(2)  A(1)    0];
end
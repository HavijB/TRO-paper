% ************************************************************************%
%                                                                         %
%                   Relative output regulation of                         %
%                   space manipulators on Lie groups                      %
%                                                                         %
%                                                                         %
% Developed by:     Borna Monazzah Moghaddam                              %
%                   Autonomous Space Robotics and Mechatronics Laboratory %
% Supervised by:    Robin Chhabra,                                        %
%                   Carleton University, Ottawa, Canada.                  %
%                                                                         %
% Initiated: 2022 August                                                  %
%                                                                         %
% Edited:                                                                 %
% ************************************************************************%

clc
close all
clear


% -------------------------------------------- Import toolboxes

% Selfmade classes

classdef Util
   properties
%       Prop
   end
   methods
      function [g] = g_Matrix(xi,qm) %#codegen
        % compute the tranformation matrix containing the rotation matrix R and
        % the translation vector s between two joints
        
        %Revolute joint
        theta=qm;
        
        v=xi(1:3);
        w=xi(4:6);
          
        % Form the g Matrix
        if norm(w)==0
            g=[eye(3) v.*qm; zeros(1,3) 1];
        else
            R=expm(skewsym(w)*theta);
            g=[R    (eye(3)-R)*skewsym(w)*v+w*transpose(w)*v.*qm; zeros(1,3) 1];
        end
        % g=inv([eye(3),b;zeros(1,3),1])*[R,s;zeros(1,3),1];
        
        end
   end
end


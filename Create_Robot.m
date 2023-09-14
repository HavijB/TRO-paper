function [] = Create_Robot()

% ************************************************************************%
%           Initiate Space-Manipulator and Target Parameters              %
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

% -------------------------------------------- Import toolboxes

% Selfmade classes


% ******************* Initiate Constants and Dimensions **************** %%
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     

n=2; %number of joints in the manipulator

% Initiate Iota the inclusion map of the base
iota0=[1 0 0; 0 1 0;zeros(3,3);0 0 1];

rho=transpose([0 0 2.5; ... % shoulder ball joint
            0 0 7] ... %elbow revolute joint
            ); %m %position of joints in initial configuration
w=transpose([1 0 0; ... % shoulder ball joint
            0 1 0;... %elbow 
            ]); %vector of rotation of each joint in inertial frame in initial pose


% form the overall twist matrix
Xi_m_matrix=zeros(6*n,n);
for i=1:n
    v(1:3,i)=-cross(w(1:3,i),rho(1:3,i));
    Xi_m_temp(1:6,i)=[v(1:3,i);w(1:3,i)];
    Xi_m_matrix(6*(i-1)+1:6*(i-1)+6,i)=[v(1:3,i);w(1:3,i)];
end

Xi_m(:,:)=Xi_m_temp;

% Set the initial poses relative to spacecraft
R(1:3,1:3,1)=[0 0 1; 0 -1 0; 1 0 0];
R(1:3,1:3,2)=[0 1 0; 0 0 1; 1 0 0];
for i=1:n
    g_bar(1:4,1:4,i)=[R(1:3,1:3,i) rho(1:3,i); 0 0 0 1];
end

% Set the initial poses of CoM of bodies in joint frames
for i=1:n
    g_cm(1:4,1:4,i)=[eye(3) [2.25;0;0]; 0 0 0 1];
end

% Set the Adjoint of initial poses of CoM of bodies in joint frames
for i=1:n
    Ad_gcm_inv(:,:,i)=inv(Adjoint(g_cm(:,:,i)));
    Ad_gbar_inv(:,:,i)=inv(Adjoint(g_bar(:,:,i)));
end

%Xi_m=0;
mu=zeros(6,1)';
% mu_t=zeros(6,1)';

m0=100;%kg
mm=[2 2]; % link1, link2, wrist masses
%temp
I0=eye(3)*416.667; %for cube with density 0.8
for i=1:n
    Im(1:3,1:3,i)=[0.00333333 0 0;0 3.37667 0; 0 0 3.37667];
end

% calculate Inertia matrices in the joint frames
[M_curly0,M_curlym]=M_curly(m0,I0,mm,Im,Ad_gcm_inv);

% ************** Initiate forces

f_0=[0;0;0];
f_m=[0;0];
f_e=[0;0;0;0;0;0];

% ************** Initiate states

q_m=[0;0];
q_dot_m=[0;0];
P=[0;0;0];
V_I0=[0;0;0];

% Form the math matrix diagonalized in the base frame

[M_frak0,M_frak] =M_frak(M_curly0,M_curlym,Ad_gbar_inv);

diag_M =diagonalize(M_frak0, M_frak);
end
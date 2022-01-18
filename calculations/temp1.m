% ************************************************************************%
%                                                                         %
%                                   Author:                               %
%                        Borna Monazzah Moghadddam                        %
%                                                                         %
%                                                                         %
%                              Simulation of a                            %
%            2-link Manipulator Mounted on a vehicle     %
%                     Based on the presented case study in:               %
%                              ...............                            %
%                                                                         %
% ************************************************************************%

clear all
clc

%% ****************** Initiate Constants and Dimensions **************** %%

%******* change into a structure that contains all robot info

% mass & inertia
m0=sym('m0');
I0=sym('I0',[3 3]);

m1=sym('m1');
I1=sym('I1',[3 3]);

m2=sym('m2');
I2=sym('I2',[3 3]);

% joint properties
l0=sym('l0');
l1=sym('l1');
l2=sym('l2');

R0I=sym('R0I',[3 3]);
p0I=sym('p0I',[3 1]);

% spacecraft
b=6; 
iota0=eye(6); 

% planar vehicle
%b=3;
%iota=[ 1 0 0; 0 1 0;zeros(3,3);0 0 1];

n=2; %number of joints
omega1=[1;0;0];
rho1=[0; 0; l0];
v1=-cross(omega1,rho1);
xi1=[omega1;v1];
xi1_hat=[skewsym(omega1) v1; 0 0 0 1];

omega2=[0;1;0];
rho2=[0; 0; l0+l1];
v2=-cross(omega2,rho2);
xi2=[omega2;v2];
xi2_hat=[skewsym(omega2) v2; 0 0 0 1];


%time-dependent
g0I=[R0I p0I; 0 0 0 1];
V0I=sym('V0I',[6,1]);
V0I_curly=iota0*V0I;

q1=sym('q1');
q2=sym('q2');



% ----------------------- Initial configurations

g0I_0=[eye(3) zeros(3,1); 0 0 0 1];

g10_0=[eye(3) [0;0;l0]; 0 0 0 1];   g1I_0=g0I_0*g10_0;

g21_0=[eye(3) [0;0;l1]; 0 0 0 1];   g20_0=g10_0*g21_0; g2I_0=g0I_0*g10_0*g21_0;

% center of mass initial (constant) configs
Rcm1=eye(3);
pcm1=[0;0;l1/2];
gcm1=[Rcm1 pcm1;0 0 0 1];

pcm2=[0;0;l2/2];
Rcm2=eye(3);
gcm2=[Rcm2 pcm2;0 0 0 1];

%% ----------------------------- Dynamics ------------------------------ %%

% --------------------- Constant properties

% inertias in joint frames
%base
M0cm_curly=[m0*eye(3) zeros(3);zeros(3) I0];
M00_curly=M0cm_curly;

%link1
M1cm_curly=[m1*eye(3) zeros(3);zeros(3) I1];

Ad_gcm1=Adjoint(Rcm1,pcm1);

M11_curly=transpose(inv(Ad_gcm1))*M1cm_curly*Ad_gcm1;

%link2
M2cm_curly=[m2*eye(3) zeros(3);zeros(3) I2];

Ad_gcm2=Adjoint(Rcm2,pcm2);

M22_curly=transpose(inv(Ad_gcm2))*M2cm_curly*Ad_gcm2;

% masses in initial configuration of base
m0_frak=M00_curly;
m1_frak=transpose(inv(Adjoint(g10_0(1:3,1:3),g10_0(1:3,4))))*M11_curly*Adjoint(g10_0(1:3,1:3),g10_0(1:3,4));
m2_frak=transpose(inv(Adjoint(g20_0(1:3,1:3),g20_0(1:3,4))))*M22_curly*Adjoint(g20_0(1:3,1:3),g20_0(1:3,4));

%twist total matrix
Xi=[iota0 zeros(6,2);zeros(6,6) xi1 zeros(6,1); zeros(6,6) zeros(6,1) xi2];
Xi_m=[xi1 zeros(6,1); zeros(6,1) xi2];

% ----------------------------- Time-evolving parameters

% L matrix: the time-dependent element
ex1=expvector(-xi1,q1); ex2=expvector(-xi2,q2); ex12=ex1*ex2;
Lm0_frak=[Adjoint(ex1(1:3,1:3),ex1(1:3,4));Adjoint(ex12(1:3,1:3),ex12(1:3,4))];
Lm_frak=[eye(6), zeros(6); Adjoint(ex2(1:3,1:3),ex2(1:3,4)) eye(6)];

% Mass Matrix
diagM12=[m1_frak zeros(6);zeros(6) m2_frak];
M0=transpose(iota0)*(m0_frak+transpose(Lm0_frak)*(diagM12)*Lm0_frak)*iota0;
M0m=transpose(iota0)*transpose(Lm0_frak)*diagM12*Lm_frak*Xi_m;
Mm=transpose(Xi_m)*transpose(Lm_frak)*diagM12*Lm_frak*Xi_m;

M0inv=inv(M0);

% reduced system

A_curly=M0inv*M0m;

Mm_hat=Mm-transpose(A_curly)*M0*A_curly;

0.5*[diff(Mm_hat,q1);diff(Mm_hat,q2)]
%for i=1:n
 %   diff(Mm_hat,q(n))
%end
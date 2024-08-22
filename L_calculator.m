function [L0,Lm0,Lm] =L_calculator(q_m,Xi_m)
% Calculates the time-dependent L matrix
n=length(q_m);
L=zeros(6*(n+1),6*(n+1));
% Compute M_frak for ith body
% Calculate L matrix
for i=1:n
    for j=1:n
        if i>j
            Ad_temp=eye(6);
            for k=i:j
                Ad_temp=Ad_temp*Adjoint(g_Matrix(-Xi_m(:,k),q_m(k)));
            end
            L(6*(i-1)+1:6*(i-1)+6,6*(j-1)+1:6*(j-1)+6)=Ad_temp;
        elseif i==j
            L(6*(i-1)+1:6*(i-1)+6,6*(j-1)+1:6*(j-1)+6)=eye(6);
        elseif i<j
            L(6*(i-1)+1:6*(i-1)+6,6*(j-1)+1:6*(j-1)+6)=zeros(6);
        end
    end
end

L0=L(1:6,1:6);
Lm0=L(7:end,1:6);
Lm=L(7:end,7:end);

end
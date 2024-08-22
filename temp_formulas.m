
n=length(q_mm);
diagM=zeros(6*n+6);
for i=1:n+1
    diagM(i*6+1:i*6+6,i*6+1:i*6+6)=inverse(transpose(Adjoint(g_cm(1:4,1:4,i))))*[mm(i)*eye(3) zero(3); zeros(3) Im(1:3,1:3,i)]*inverse(Adjoint(g_cm(1:4,1:4,i)));
end

for i=1:n+1
    m_frak(1:6,1:6,i)=inverse(transpose(Adjoint(g_bar(1:4,1:4,i))))*(diagM(i*6+1:i*6+6,i*6+1:i*6+6))*inverse(Adjoint(g_bar(1:4,1:4,i)));
end

Mm_hat_partial(i)=Mm_partial(i)-; % Equation 119 of TRO paper

Mm_partial=transpose(Xi_m)*(transpose(Lm_partial(i))Lm + ...
    transpose(Lm)*()*Lm_partial(i))*Xi_m; %calcualte the derivative of not-augmented arm mass matrix from equation 120 of TRO paper

A_partial(i)=inverse(M0)*M0_partial(i)*inverse(M0)*M0m+inverse(M0)*M0m_partial(i);

for i=1:n
    M0_partial(i)= transpose(Lm0_partial(i))*diagM*Lm0+transpose(Lm0)*diagM*Lm0_partial(i); %6x6 %equation 117 of TRO
end
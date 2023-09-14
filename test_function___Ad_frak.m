% Ad_frak Test function

% run main setup
% run(Main)

for i=1:n
    for j=1:i
        ans(:,:,i)=Ad_frak(i,j,Xi_m,q_m);
    end
end
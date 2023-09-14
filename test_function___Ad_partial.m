% Ad_frak Test function

% run main setup
% run(Main)
clear ans

for i=1:n
    for k=1:n
        for j=1:k
            ans(:,:)=Ad_partial(j,k,i,Xi_m,q_m,iota0)
        end
    end
end
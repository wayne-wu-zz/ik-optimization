function p = penaltyfunc_fminuncon(chain,x)
%PENALTYFUNC Penalty function

p = 0;
lb = chain.lb;
ub = chain.ub;
n = chain.n;
k = chain.k;

for link = 1:n
    if (x(link) < lb(link)) || (x(link) > ub(link)) % if x is outside bound interval
        p = p + k*x(link)*x(link)*x(link)*x(link); % penalty increase
    end
end

end


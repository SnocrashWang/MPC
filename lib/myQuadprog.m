function [y] = myQuadprog(A,b)
x0=zeros(size(b));
r0 = A*x0 + b;
p0 = r0;
for i = 1:1000
    alpha = (r0'*r0)/(p0'*A*p0);
    x = x0 - alpha*p0;
    r = r0 - alpha*A*p0;
    beta = (r'*r)/(r0'*r0);
    p = r + beta*p0;
    if norm(x-x0)<=10^(-8)
        y=x(1);
        break
    end
    x0 = x;
    r0 = r;
    p0 = p;
end

if y > 12
    y = 12;
elseif y < -12
    y = -12;
end
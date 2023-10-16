function u_k= predict(x_k,E,E_,H,N,p,Xd)

xd=reshape(Xd,[(N+1)*size(Xd,1),1]);
% U_k = quadprog(H,E*x_k-E_*xd, [], [], [], [], ones(5, 1) * (-4*pi - x_k(2)), ones(5, 1) * (4*pi - x_k(2)));
U_k = myQuadprog(H,E*x_k-E_*xd);
u_k = U_k(1:p,1); % 取第一个结果

end
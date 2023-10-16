function [E, E_, H]=getMatrices(A,B,Q,R,F,N)

n=size(A,1); % A �� n x n ����, �õ� n
p=size(B,2); % B �� n x p ����, �õ� p

%%%%%%%%%%%%
M=[eye(n);zeros(N*n,n)]; % ��ʼ�� M ����. M ������ (N+1)n x n�ģ��0�2
% �������� n x n �� "I", ��һ���Ȱ��°벿��д�� 0
C=zeros((N+1)*n,N*p); % ��ʼ�� C ����, ��һ�������� (N+1)n x NP �� 0

% ����M �� C�0�2
tmp=eye(n); %����һ��n x n �� I ����

%�����£ͺ�C
for i=1:N % ѭ����i �� 1�� N
    rows =i*n+(1:n); %���嵱ǰ��������i x n��ʼ����n��
    C(rows,:)=[tmp*B,C(rows-n, 1:end-p)]; %��c��������
    tmp= A*tmp; %ÿһ�ν�tmp���һ��A
    M(rows,:)=tmp; %��M����д��
end

% ����Q_bar��R_bar
Q_bar = kron(eye(N),Q);
Q_bar = blkdiag(Q_bar,F);
R_bar = kron(eye(N),R);

% ����G, E, H
G=M'*Q_bar*M; % G: n x n
E=C'*Q_bar*M % E: NP x n
E_=C'*Q_bar
H=C'*Q_bar*C+R_bar % NP x NP�0�2


end

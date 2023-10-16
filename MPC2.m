%% 清屏
clc;
clear;
close all;
addpath(genpath('.\lib'));

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 第一步，定义状态空间矩阵
% 定义状态矩阵 A, n x n 矩阵
A = [1 0.01; 0 1];
n = size(A,1);

% 定义输入矩阵 B, n x p 矩阵
B = [0; 1];
p = size(B,2);

% 定义Q矩阵，n x n 矩阵
Q=[10000 0;0 1];
% 定义F矩阵，n x n 矩阵
F=[10000 0;0 1];
% 定义R矩阵，p x p 矩阵
R=[1];

% 定义step数量k
k_steps=100;

% 定义矩阵 X_K， n x k 矩 阵
X_K = zeros(n,k_steps);
% 初始状态变量值， n x 1 向量
X_K(:,1) =[0;0];

% 定义输入矩阵 U_K， p x k 矩阵
U_K=zeros(p,k_steps);

% 定义目标跟随状态量
Xd = getTargetYaw(100, 1, 1, 1, -1, 0.25, 0, 6);

% 定义预测区间N
N=5;

% 求E,H矩阵
[E,E_,H]=getMatrices(A,B,Q,R,F,N);

%% 计算每一步的状态变量的值
for k = 1 : k_steps-N
    % 求U_K(:,k)
    U_K(:,k) = predict(X_K(:,k),E,E_,H,N,p,Xd(:,k:k+N));
    % 计算第k+1步时状态变量的值
    X_K(:,k+1)=(A*X_K(:,k)+B*U_K(:,k))+randn(2,1)*0.001;
end

%% 绘制状态变量和输入的变化
subplot(3, 1, 1);
hold;
plot((1:k_steps)/100, X_K(1,:)),axis([0 1 -0.5 1.5]);
plot((1:k_steps)/100, Xd(1,:)),axis([0 1 -0.5 1.5]);
legend("current yaw","target yaw")
hold off;

subplot(3, 1, 2);
hold;
plot((1:k_steps)/100, X_K(2,:)),axis([0 1 -5 15]);
plot((1:k_steps)/100, Xd(2,:)),axis([0 1 -5 15]);
legend("control speed","standard speed")
hold off;

subplot(3, 1, 3);
plot((1:k_steps)/100, U_K(1,:)),axis([0 1 -15 15]);
legend("diff speed")

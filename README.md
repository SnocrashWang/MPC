# README

本仓库代码用于测试模型预测控制算法

## 仓库结构

```
.
├─ MPC1.m                          # 主程序，使用位置作为状态量
├─ MPC2.m                          # 主程序，使用位置和速度作为状态量
└─ lib
    ├─ getTargetYaw.m              # 获取数据序列
    ├─ getMatrices.m               # 计算MPC过程中间矩阵
    ├─ myQuadprog.m                # 无约束二次规划求解器
    └─ predict.m                   # 求解控制量
```

## 算法原理

### 模型预测控制

假设有离散状态空间方程如下，其中 $x_d(k)$ 为 $k$ 时刻的目标控制量。
$$
x(k+1) = A x(k) + B u(k) \\
e(k) = x_d(k) - x(k)
$$
预测区间
$$
X(k) =
\begin{pmatrix}
x(k|k) \\
x(k+1|k) \\
\vdots \\
x(k+N|k) \\
\end{pmatrix}
$$
控制区间
$$
U(k) =
\begin{pmatrix}
u(k|k) \\
u(k+1|k) \\
\vdots \\
u(k+N-1|k) \\
\end{pmatrix}
$$
目标跟随区间
$$
X_d(k) =
\begin{pmatrix}
x_d(k|k) \\
x_d(k+1|k) \\
\vdots \\
x_d(k+N|k) \\
\end{pmatrix}
$$


预测区间与控制区间满足如下关系
$$
X(k) = M x(k) + C U(k)
$$
其中
$$
M =
\begin{pmatrix}
I \\
A \\
A^2 \\
\vdots \\
A^N
\end{pmatrix},
C =
\begin{pmatrix}
\bold0 & \bold0 & \cdots & \bold0 \\
B & \bold0 & \cdots & \bold0 \\
AB & B & \cdots & \bold0 \\
\vdots & \vdots & \ddots & \bold0 \\
A^{N-1}B & A^{N-2}B & \cdots & B
\end{pmatrix}
$$
损失函数
$$
\begin{align}
J &= \sum_{i=0}^{N-1} {\left( e(k+i|k)^T Q e(k+i|k) + u(k+i|k)^T R u(k+i|k) \right)} + e(k+N|k)^T F e(k+N|k) \\
  &= (X_d(k) - M x(k))^T \overline{Q} (X_d(k) - M x(k)) - 2(X_d(k) - M x(k))^T \overline{Q} C U(k) + U(k)^T (C^T \overline{Q} C + \overline{R}) U(k) \\
J^- &= U(k)^T H U(k) + 2(E x(k) - E_d X_d(k))^T U(k)
\end{align}
$$
其中
$$
\begin{align}
E &= C^T \overline{Q} M \\
E_d &= C^T \overline{Q} \\
H &= C^T \overline{Q} C + \overline{R} \\
\overline{Q} &=
\begin{pmatrix}
Q & & & \\
& Q & & \\
& & \ddots & \\
& & & F
\end{pmatrix} \\
\overline{R} &=
\begin{pmatrix}
R & & \\
& \ddots & \\
& & R
\end{pmatrix}
\end{align}
$$

### 二次规划

MPC 的本质是求解一个二次规划问题，它的一般形式如下
$$
\begin{align}
\min & \quad F(x) = \frac12 x^T H x + f^T x \\
s.t. & \quad A \cdot x \le b \\
	 & \quad Aeq \cdot x = beq \\
	 & \quad lb \le x \le ub
\end{align}
$$

无约束二次规划问题采用共轭梯度法求解
$$
\begin{align}
&\text{set} \quad r_0 = A x_0 + b, \quad p_0 = r_0 \\
&\begin{array}
\text{repeat} \quad
&\alpha_k = \frac{r_k^T r_k}{p_k^T A p_k} \\
&x_{k+1} = x_k - \alpha_k p_k \\
&r_{k+1} = r_k - \alpha_k A p_k \\
&\beta = \frac{r_{k+1}^T r_{k+1}}{r_k^T r_k} \\
&p_{k+1} = r_{k+1} + \beta_k p_k
\end{array}
\end{align}
$$

## 实验效果

在一个简单的 CV 模型中，我们测试了两种不同状态空间方程：一种仅以位置为状态量，以速度为控制量；另一种以位置和速度为状态量，以速度与目标速度的差分为控制量。前者为简单计算，在此不赘述，后者的状态空间方程如下所示
$$
\begin{pmatrix}
x_{k+1} \\
\dot{x}_{k+1}
\end{pmatrix}
=
\begin{pmatrix}
1 & \Delta t \\
0 & 1
\end{pmatrix}
\begin{pmatrix}
x_k \\
\dot{x}_k
\end{pmatrix}
+
\begin{pmatrix}
0 \\
1
\end{pmatrix} u_k
$$

> 我知道拿速度差分当控制量非常抽象，但是速度和电流的关系式有些过于复杂，我暂且还是把这个问题留给电控的 PID，或者让有志后人来解决它吧。

两种方法的跟随效果基本无异，在运动状态稳定变化和突变时均有良好的的响应，可以看到有速度作为状态量时系统的响应稍快一筹。MPC 对比 PID 的主要优势便在于调参的简易性和在不同运动情况下的鲁棒性。

这里再着重解释一下 MPC 的参数，$Q$ 代表对预测时间段内误差的重视程度，$R$ 代表对预测时间段内控制量的重视程度，$F$ 表示对预测时间段后的这一特定时刻的误差量的重视程度。

---

## 参考资料

[模型预测控制器，DR_CAN，Bilibili](https://www.bilibili.com/video/BV1cL411n7KV/?share_source=copy_web&vd_source=1f484e59e99ed29fdece5126993aa064)


clear
clc
A_c=[1 0.01;...
    -0.1 1-0.01];
B_c=[   0;   0.01];
Qx=100*diag([1,0.1]);
Qu=0.1*eye(1);

J = 10;
b = 0.2;
A=[0 1 ;
        0 -b/J];
B = [0 ; 1/J];
[A_c,B_c]=c2d(A,B,0.01);

A_tot=A_c;
B_tot=B_c;
[P,K,G] = dare(A_tot,B_tot,Qx,Qu);
% K_l2 是最优化求得的增益	K_l2=[-21.3490	-10.7757]
K_l2=-inv(Qu+B_tot'*P*B_tot)*B_tot'*P*A_tot
% K_l2=[-21.3490	-10.7757];


dt=0.01;
T=0:dt:5;

theta = zeros(6,1);

xu = zeros(3,1);
xu_pre = zeros(3,1);

gamma = 1;
Kq = 0.1*K_l2;
K_save = Kq;
cost= 0;

noise_trial=20;		%噪声的大小
noise_dis=[0 0.]';
x_save = [];
mu_boundary=10*noise_trial;
for eps=1:1:10
    
    
    %回到起始位置
    y0 = [0.5 0.5]';

    P_ = eye(6)*1000;
    
    for i = 1:length(T)
        
        x_save = [x_save y0];
        u = Kq*y0+noise_trial*(10+2-eps)/10*((randn));   %计算输出，加上探索噪声
        
        if u>mu_boundary || u<-mu_boundary
            u=sign(u)*mu_boundary;
        end
        
        % 作用于系统
        noise=noise_dis*randn;
        y1 = A_c*y0+B_c*u+noise;
        
        
        % 通过最小二乘法估计Q函数的参数以及Theta的值
        cost = y0'*Qx*y0+u*Qu*u;   
        
        xu_pre=[u ;y0];
        xu = [ Kq*y1;  y1];
        phi_all = kron(xu_pre,xu_pre) - gamma*kron(xu,xu);
        phi = phi_all([1 2 3 5 6 9]);
     
        temp_ = 1+phi'*P_*phi;
        
        gradient_ =  P_*phi*(cost-phi'*theta)/temp_;
        
        theta = theta + gradient_;
        
        P_ = P_-(P_*phi*phi'*P_)/temp_;
        
        y0 = y1;
        
    end
    
    % 更新控制律 U
     H22_ = theta(1,1);
     H21_ = theta(2:3,1)/2;
     Kq = -(H21_/H22_)'
%     Kq = -inv(H_22)*H_21
    
    K_save = [K_save;Kq];
    % 绘制曲线
    figure(10)
    plot(T,x_save(1,:));
    drawnow();
    pause(1)
    x_save=[];
    
    
end
i = 1:1:length(K_save);
figure(20)
plot(i,K_save)

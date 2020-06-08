
clc
clear
% 定义系统的方程
J = 10;
b = 0.2;
A = [0 1 ;
        0 -b/J];
B = [0 ; 1/J];

C = [1 0];

% %LQR控制器
Q = C'*C ;
Q(1,1) = 100;
Q(2,2) = 10;
R = 0.1; 
K = lqr(A,B,Q,R) ;

Ac = (A-B*K); 
Bc = B; 

y0 = [-pi/6 0];
tspan = [0 10];
[t,y]=ode45(@(t,y) odefcn(t,y,Ac,zeros(2,1)),tspan,y0);
figure(2)
plot(t,y(:,1));
title(['K1= ',num2str(K(1)),'    K2= ',num2str(K(2))]);

%Q-Learning 部分
%定义学习参数，Qx是对称正定矩阵，定义cost
Qx = [10 0;
           0 1];
% Qx = Q;
Qu = ones(1,1);

Kq = -[3.1623 5.4270];

% Kq = [0 0];
K_save = Kq;
u = 0;
cost = 0;

theta = zeros(6,1);
xu = zeros(3,1);
xu_pre = zeros(3,1);
gamma = 0.85;
u_save = [];
uc_save=[];
cost_save = [];

figure(1)
t = 1:0.01:5;
y = exp(t);
plot(t,y);
% for eps=1:1:10
%     %回到起始位置
%     y0 = [0.5 0.5];
%     t_save = 0;
%     y_save = y0;
%     
%     P_ = eye(6)*1000;
%     
%     for i = 1:1:(5/0.01)
%         
%         u = Kq*y0';
%         uc_save=[uc_save u];
%         u=u+10*(10+2-eps)/10*(rand()-0.5);   %计算输出，加上探索噪声
%         u_save=[u_save u];
%         % 作用于系统
%         Bu= B*u;
%         tspan=[0 0.01];    %控制周期为8ms
%         [t,y] = ode45(@(t,y) odefcn(t,y,A,Bu),tspan,y0);
%         t_save = [t_save (t_save(end)+t)'];
%         y_save = [y_save;y];
%         y1 = y(end,:);
%         
%         if(abs(y1(1))>50)   %阈值限定
%             check = 0
%             break;
%         end
%         
%         % 通过最小二乘法估计Q函数的参数以及Theta的值
%         
%         cost = y0*Qx*y0'+u*Qu*u;   
%         cost_save=[cost_save cost];
%         
%         xu_pre=[u y0]';
%         xu = [ Kq*y1'  y1]';
%         phi_all = kron(xu_pre,xu_pre) - gamma*kron(xu,xu);
%         phi = phi_all([1 2 3 5 6 9]);
%      
%         temp_ = 1+phi'*P_*phi;
%         
%         gradient_ =  P_*phi*(cost-phi'*theta)/temp_;
%         
%         theta = theta + gradient_;
%         
%         P_ = P_-(P_*phi*phi'*P_)/temp_;
%         
%         y0 = y1;
%     end
%     
%     % 更新控制律 U
%      H22_ = theta(1,1);
%      H21_ = theta(2:3,1)/2;
%      Kq = -(H21_/H22_)'
% %     Kq = -inv(H_22)*H_21
%     
%     K_save = [K_save;Kq];
%     % 绘制曲线
%     figure(10)
%     plot(t_save,y_save(:,1));
%     title("state");
%     figure(30)
%     i = 1:1:length(u_save);
%     plot(i,u_save(1,:),'b');
%     title("output u ");
%     hold on
%     plot(i,uc_save(1,:),'r');
%     hold off
%     u_save=[];
%     uc_save=[];
%     figure(40)
%     i = 1:1:length(cost_save);
%     plot(i,cost_save(1,:));
%     title("cost");
%     cost_save=[];
%    drawnow();
%     pause(1)
% end
% i = 1:1:length(K_save);
% figure(20)
% plot(i,K_save);
% title("gain");


function dydt=odefcn(t,y,A,Bu)
dydt = zeros(2,1);
dydt(1)=A(1,1)*y(1)+A(1,2)*y(2);
dydt(2)=A(2,1)*y(1)+A(2,2)*y(2);
% for i=1:1:2
%     for j = 1:1:2
%     dydt(i) = dydt(i)+A(i,j)*y(j);
%     end
%     dydt(i)=dydt(i)+Bu(i);
% end
end






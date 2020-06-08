function TD_ac_control
close all
clc
%%% System description:
dim_A=2;
dim_B=1;
A_c=[1 0.01;...
    -0.1 1-0.01];
B_c=[   0;   0.01];

J = 10;
b = 0.2;
A=[0 1 ;
        0 -b/J];
B = [0 ; 1/J];
[A_c,B_c]=c2d(A,B,0.01);


Q_1=100*diag([1,0.1]);
R=0.1*eye(1);

% Q_1=10*diag([1,0.1]);
% R=1*eye(1);

% A_c=[0 0.1;0.3 -1];
% B_c=[0;1];
% Q_1=3*[1 0;0 1];
% R=1*eye(1);
A_tot=A_c;
B_tot=B_c;
[P,K,G] = dare(A_tot,B_tot,Q_1,R);
K_l2=-inv(R+B_tot'*P*B_tot)*B_tot'*P*A_tot
dt=0.01;
T=0:dt:5;

%%% initialization:
Num_theta=sum(1:dim_A+dim_B);
theta_i=zeros(Num_theta,1);
V_save=[];
K_x_V=[];
K_error=[];
theta_save=[];
K_l=0.1*K_l2;zeros(1,dim_A);
K_l_AC=zeros(1,dim_A);

%%% learning parameters
Num_ep=10;
noise_trial=20;
noise_dis=[0 0.]';
mu_boundary=10*noise_trial;


gamma=1;
P_i_level=1e5;
updata_max=100;
X_k0=[.5 .5]';
X_k_save=[];

for j=1:Num_ep
    P_i=eye(Num_theta)*P_i_level;
    X_k_V=[X_k0];    
    J_sum=zeros(1,dim_A);
    G_sum=zeros(1,dim_A);
    
    for i=1:length(T)
        X_k_save=[X_k_save X_k_V];
        X_k_V_=X_k_V;
        mu_V=K_l*X_k_V_+noise_trial*(Num_ep+2-j)/10*((randn));
        if mu_V>mu_boundary || mu_V<-mu_boundary
            mu_V=sign(mu_V)*mu_boundary;
        end
        
        noise=noise_dis*randn;
        X_k_V=A_c*X_k_V_+B_c*mu_V+noise;
        
        c_k=X_k_V_'*Q_1*X_k_V_ + mu_V'*R*mu_V;   %
        
%         phi_e=gradient_Q(X_k_V_,X_k_V,mu_V,K_l,gamma)
        
        
%         xu_pre=[mu_V ; X_k_V_];
%         xu = [ K_l*X_k_V;  X_k_V];
         xu_pre=[ X_k_V_;mu_V];
         xu = [  X_k_V; K_l*X_k_V];
        phi_all = kron(xu_pre,xu_pre) - gamma*kron(xu,xu);
        phi_e = phi_all([1 2 3 5 6 9]);
        
        
        gradient=P_i*phi_e*(c_k-phi_e'*theta_i)/(1+phi_e'*P_i*phi_e);
        theta_i=theta_i+gradient;
        P_i=P_i-P_i*phi_e*phi_e'*P_i/(1+phi_e'*P_i*phi_e);
        
        [H_21,H_22]=H_2_theta(X_k_V_,mu_V,theta_i); % reshape the theta to H,
        
         H_22_ = theta_i(end,1);
         H_21_ = theta_i([3,5])'/2;
                 
        
        
        update_J_sum=0.0005;
        K_l_AC=K_l_AC-update_J_sum*J_sum; %
        K_l_Q=-inv(H_22)*H_21;
        
        J_sum=20*((H_21+H_22*K_l_AC));        
        
%         if norm(J_sum)>updata_max
%             J_sum=J_sum/norm(J_sum)*updata_max;
%         end
        
        Swith_Q2ac=11;
        alpha_Q=0.05;
        update_step=50;
        if i>update_step*1 && mod(i,update_step)==0
            if j<Swith_Q2ac
                K_l=K_l_AC;
            else
                K_l=alpha_Q*K_l_Q+(1-alpha_Q)*K_l;
            end
        end
        
        V_save=[V_save; (c_k).^2];
        mu_save(:,i)=mu_V;
        
        theta_save=[theta_save theta_i];
        K_x_V=[K_x_V; K_l];
        K_error=[K_error norm(K_l2-K_l)];
    end
end

K_l

figure(23),hold on,plot([1:length(X_k_save)]*0.04,X_k_save(1,:),'r'),title('K save');

figure(30),hold on,plot(theta_save','r'),title('theta save')

figure(21),hold on,plot(K_x_V),%plot(sum(abs(K_x_V-K_l2),2))
plot((K_l2'*ones(1,length(K_x_V)))'),title('K_I2')
xlabel('iteration step'),ylabel('control value')

figure(2),hold on,plot(K_error/norm(K_l2)),title('K_error/norm(K_I2)');

verification(A_c,B_c,K_l2,Q_1,R,X_k0,T)
verification(A_c,B_c,K_l,Q_1,R,X_k0,T)
%contour_map(A_tot,B_tot,K_l2,R,Q_1,P) % gradient map
end

function [H_21,H_22]=H_2_theta(X_k_V_,mu_V,theta_i)
L_num=length(X_k_V_)+length(mu_V);
theta_i_new=theta_i;
N_num=0;
for i=1:L_num-1
    N_num=N_num+L_num+1-i;
    H_21(1,i)=theta_i_new(N_num)/2;
end
H_22=[theta_i_new(end)];
end

function phi_e=gradient_Q(X_k_V_,X_k_V,mu_V,K_l,gamma)
H_xx_=[X_k_V_;mu_V];
H_xx=[X_k_V;K_l*X_k_V];
L_num=length(H_xx_);
phi_e_all=[kron(H_xx_,H_xx_)-gamma*kron(H_xx,H_xx)];
Num_Q=1;
for ii=1:L_num
    for jj=1:L_num
        if jj>=ii
            phi_e(Num_Q,1)=phi_e_all((ii-1)*L_num+jj);
            Num_Q=Num_Q+1;
        end
    end
end

end

function contour_map(A_tot,B_tot,K_l2,R,Q_1,P)
K_range_x=-20:0.5:0;
K_range_y=-10:0.5:20;
K_test=K_l2;
for ii=1:length(K_range_x)
    for jj=1:length(K_range_y)
        K_test(2)=K_range_x(ii);
        K_test(4)=K_range_y(jj);
        P_cal=-pinv(B_tot)'*R*K_test*inv(A_tot+B_tot*K_test);
        V_map(ii,jj)=norm(diag(Q_1-P_cal+A_tot'*P_cal*A_tot+A_tot'*P*B_tot*K_test),2);
        XX_K(ii,jj)=K_test(2);
        YY_K(ii,jj)=K_test(4);
    end
end
figure(20),contour(XX_K,YY_K,V_map)
end

function verification(A_c,B_c,K_l2,Q_1,R,X_k0,T)
X_k=X_k0;
X_k_V2=[X_k];
for i=1:length(T)
    X_k_V_save2(:,i)=X_k_V2;
    u_k=K_l2*X_k_V2;
    X_k_V2=A_c*X_k_V2+B_c*u_k;
    V(i)=X_k_V2'*Q_1*X_k_V2+u_k'*R*u_k;
end
figure(2001),hold on,plot(X_k_V_save2')
xlabel('time'),ylabel('value')
sum(abs(V)) % best value=8.8309e+04
end
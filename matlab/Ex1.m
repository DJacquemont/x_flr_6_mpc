%% Setup
clear;
% System dynamics
A = [4/3 -2/3;1 0];
B = [1;0];
C = [-2/3 1];

% Cost matrices
Q = C'*C+0.001*eye(2);
R = 0.001;

%% Prob1 
% Horizon
N = 8;
Kcomp = zeros(size(A,1),N);
H = Q;

for i = N-1:-1:0
    disp(i);
    K = -(R+B'*H*B)\B'*H*A;
    H = Q + K'*R*K + (A + B*K)'*H*(A+B*K);

    Kcomp(:,i+1) = K;
end

%% Prob2

x0 = [52;103];
x(:,1) = x0;
tmax = 5;

%V0 = x'*H*x;
%U0 = K*x;

% Plotting the state evolution
figure(2)
plot(x0(1),x0(2),'rx'); 
axis([0 11 0 11]); hold on; grid on;

for t= 1:tmax
    
    H = Q;
    
    for i = N-1:-1:0
        disp(i);
        K = -(R+B'*H*B)\B'*H*A;
        H = Q + K'*R*K + (A + B*K)'*H*(A+B*K);
    
        Kcomp(:,i+1) = K;
    end

    u = K*x(:,t);
    x(:,t+1) = A*x(:,t) + B*u;

end

plot(x(1,:),x(2,:),'k-o','LineWidth',1.2);

xpred(:,1) = x0;
for t=1:N
    xpred(:,t+1) = A*xpred(:,t) + B*(Kcomp(:,t)'*xpred(:,t)); % fixed
end 
%plot(xpred(1,:),xpred(2,:),'m-o','LineWidth',1.2);
legend('Initial condition','Closed-loop evolution','Prediction');
title('State-Space');


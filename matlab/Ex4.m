clear;
%% initialization
A = [0.9752 1.4544;
    -0.0327 0.9315];

B = [0.0248;
     0.0327];

x0 = [3; 0];
Q = 10 * eye(2);
R = 1;

% Constraints
% u in U = { u | Mu <= m }
M = [1;-1]; m = [1.75; 1.75];
% x in X = { x | Fx <= f }
F = [1 0; 0 1; -1 0; 0 -1]; f = [5; 0.2; 5; 0.2];

% Compute LQR controller for unconstrained system
[K,Qf,~] = dlqr(A,B,Q,R);
% MATLAB defines K as -K, so invert its signal
K = -K; 

% Compute maximal invariant set
Xf = polytope([F;M*K],[f;m]);
Acl = [A+B*K];
while 1
    prevXf = Xf;
    [T,t] = double(Xf);
    preXf = polytope(T*Acl,t);
    Xf = intersect(Xf, preXf);
    if isequal(prevXf, Xf)
        break
    end
end
[Ff,ff] = double(Xf);

% MPT version
sys = LTISystem('A',A,'B',B);
sys.x.min = [-5; -0.2]; sys.x.max = [5; 0.2];
sys.u.min = [-1.75]; sys.u.max = [1.75];
sys.x.penalty = QuadFunction(Q); sys.u.penalty = QuadFunction(R);

Xf_mpt = sys.LQRSet;
Qf_mpt = sys.LQRPenalty;

% check by yourself if they are equal...

% Visualizing the sets
%figure
%hold on; grid on;
%plot(polytope(F,f),'g'); plot(Xf,'r');
%xlabel('position'); ylabel('velocity');



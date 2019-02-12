
load('IdentifyedModelDutyToTranslationalVelocity.mat');
load('IdentifyedModelDutyToRotationalVelocity.mat');

P = ss(IdentifyedModelDutyToTranslationalVelocity);
% [V D] = eig(P.A);
% P = canon(P, 'companion');
% P = ss2ss(P, P.A(1, 2)*eye(2));
% P = ss2ss(P, [1 0;0 P.A(1, 2)]);
% P = ss2ss(P, P.C(2)*eye(2));
% P = ss2ss(P, [0 1;1 0]);
% P

sys = P;
[n,d]=tfdata(tf(sys),'v');
W=hankel(fliplr(d(1:numel(d)-1)));
G=ss(sys);
UC=ctrb(G);
S=UC*W;
csys=ss(inv(S)*G.a*S,inv(S)*G.b,G.c*S,G.d);
csys = ss2ss(csys, csys.C(1)*eye(2));
csys

A = csys.A;
B = csys.B;
C = csys.C;
CAB = C*A*B;

CAB \ 1
CAB \ (-C*A*A)

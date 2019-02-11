syms x y th
syms dx dy dth
syms ddx ddy ddth
syms v w
syms dv dw


k1 = 10;
k2 = 10;
Gx = tf([1 k1], [1 k1 k2]);
step(Gx);

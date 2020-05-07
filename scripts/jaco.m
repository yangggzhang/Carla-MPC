syms v psi lr lf delta a L x y
f = [ v*cos(psi + atan(lr/L*tan(delta))); v*sin(psi + atan(lr/L*tan(delta))); a; v/lr*sin(atan(lr/L*tan(delta)))];
A = jacobian(f, [x; y; v; psi]);
B = jacobian(f, [a; delta]);
X = [x; y; v; psi];
U = [a; delta];
C = f - A*X - B*U;
syms v psi lr lf delta a L x y
A = jacobian([ v*cos(psi + atan(lr/L*tan(delta))), v*sin(psi + atan(lr/L*tan(delta))), a, v/lr*sin(atan(lr/L*tan(delta)))], [x, y, v, psi]);
B = jacobian([ v*cos(psi + atan(lr/L*tan(delta))), v*sin(psi + atan(lr/L*tan(delta))), a, v/lr*sin(atan(lr/L*tan(delta)))], [a, delta]);
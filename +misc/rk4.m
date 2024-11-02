function y = rk4(f,x,u,h)
    k1 = h * f(x,u);
    k2 = h * f(x + k1/2,u);
    k3 = h * f(x + k2/2,u);
    k4 = h * f(x + k3,u);
    
    y = x + (1/6)*(k1 + 2*k2 + 2*k3 + k4);
end
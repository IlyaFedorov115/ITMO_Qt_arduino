function [dx,y] = Pend1(t,x,u,m,g,L,d,c,J, K, varargin)
y = x(1);
dx = [x(2); 
    -m*g*d*sin(x(1))/J - c*x(2)/J + K*L*u/J];
end
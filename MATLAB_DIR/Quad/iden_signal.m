clc, clear, close all;


u = idinput(101,'prbs',[0 1],[-10 10]);
d = diff(u);
idx = find(u) + 1;
idx = [1;idx];
for ii = 1:length(idx) - 1
     amp = randn;
     u(idx(ii):idx(ii+1)-1) = amp*u(idx(ii));
end

%%
u = u/max(u);
u = u*20;
u = iddata([],u,1);
% Plot the data
figure
plot(u)

res = [u.InputData]';
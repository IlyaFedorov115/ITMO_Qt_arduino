%clc, clear, close all;


u = idinput(101,'prbs',[0 1],[-15 20]);
d = diff(u);
idx = find(u) + 1;
idx = [1;idx];
for ii = 1:length(idx) - 1
     amp = randn;
     u(idx(ii):idx(ii+1)-1) = amp*u(idx(ii));
end


%u = u/max(u);
u = 1450 + u*20;
%u = 1450 + u;
u = rescale(u, 1437, 1474);
u = iddata([],u,5);
% Plot the data
figure
plot(u)

res = [u.InputData]';

%%
%%
NumChannel = 1;
Period = 15;
NumPeriod = 5;
u = idinput([Period, NumChannel, NumPeriod], 'rgs');
u = iddata([], u, 1);
figure
plot(u)
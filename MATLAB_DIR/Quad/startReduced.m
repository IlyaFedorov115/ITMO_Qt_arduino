%% очистка
clear all
close all


%% Константы
% изменения или ненужное из старой
%tau_y = 10; % сила прикладываемая к вертикальному движению

% входы
y_d = 0.2; % высота (задающее воздействие по оси 'OY')
gamma_d = 0.1;
theta_d = -0.1;
psi_d = 0.1;

% физические параметры
l = 0.225; % расстояние от центра масс до двигателей

g = 9.81; 
m = 1.44 ;  % масса квадрокоптера
A_x = 0.25; % аэродинамический коэффициент вдоль оси 'OX' кг/с
A_y = 0.25; % аэродинамический коэффициент вдоль оси 'OY'
A_z = 0.25; % аэродинамический коэффициент вдоль оси 'OZ'

omega_r = 4; % внешний сигнал, который подадим чуть позже

%% для угловых скоростей
kl = 3 * 10^(-6);

b = 1.14 * 10^(-7);
a_r = 0.033; % постоянная вращательного движения
m_s_my = [1/(kl*4) 0 -1/(2*l*kl) -1/(4*a_r); 1/(kl*4) -1/(2*l*kl) 0 1/(4*a_r); 1/(kl*4) 0 1/(2*l*kl) -1/(4*a_r); 1/(kl*4) 1/(2*l*kl) 0 1/(4*a_r)];
m_s = [1/4 0 -1/(2*l) -1/(4*a_r); 1/4 -1/(2*l) 0 1/(4*a_r); 1/4 0 1/(2*l) -1/(4*a_r); 1/4 1/(2*l) 0 1/(4*a_r)];
%% параметры регуляторов для модели с кватернионом

K_y_D = 13.58; % коэффициент  общей тяги        !!! Совпадает со статье
K_y_P = 33.18; % коэффициент  общей тяги        !!! Совпадает со статье

P_q1 = 20; % коэффициент пропорциональной составляющей для мнимого единичного вектора i
P_q2 = 20; % коэффициент пропорциональной составляющей для мнимого единичного вектора j
P_q3 = 20; % коэффициент пропорциональной составляющей для мнимого единичного вектора k
P_q = diag([P_q1 P_q2 P_q3]);

P_omega_x = 4; % коэффициент пропорциональной составляющей скорости вращения в доль оси 'OX'
P_omega_y = 4; % коэффициент пропорциональной составляющей скорости вращения в доль оси 'OY'
P_omega_z = 4; % коэффициент пропорциональной составляющей скорости вращения в доль оси 'OZ'
P_omega = diag([P_omega_x P_omega_y P_omega_z]);

Ix = 0.0151;
Iy = 0.0151;    % может быть из-за ZYX???
Iz = 0.0253;
Icm = diag([Ix Iy Iz]);

dt = 1e-4;

initial_angles = [0, 0, 0];
initial_quaternion = quaternion(initial_angles, 'euler', 'ZYX', 'frame');

task_angles = [gamma_d, psi_d, theta_d];
task_quaternion = quaternion(task_angles, 'euler', 'ZYX', 'frame');

%% параметры регуляторов для модели с ПД-регулятором
I_x = 0.0151; % момент инерции квадрокоптера при его вращении вдоль оси 'OX' 
I_y = 0.0253; % момент инерции квадрокоптера при его вращении вдоль оси 'OY' 
I_z = 0.0151; % момент инерции квадрокоптера при его вращении вдоль оси 'OZ' 
I_r = 5.38 * 10^(-5); % инерция ротора двигателя

K_gamma_D = 0.14;  %'OX' 10.4 / 36
K_gamma_P = 0.35; %  'OX' 22 / 80

K_theta_D = 0.14;   %  'OZ' 11.8
K_theta_P = 0.35;  %  'OZ' 24

K_psi_D = 0.24;   %  'OY' 25
K_psi_P = 0.58;  %  'OY' 95

%K_gamma_D = 10;  %'OX' 10.4 / 36
%K_gamma_P = 40; %  'OX' 22 / 80

%K_theta_D = 10;   %  'OZ' 11.8
%K_theta_P = 40;  %  'OZ' 24

%K_psi_D = 10;   %  'OY' 25
%K_psi_P = 40;  %  'OY' 95

K_gamma_P1 = 1.79;
K_gamma_D1 = 0.28539;
K_gamma_I1 = 3.7751;


K_gamma_P1 = 1.85548;
K_gamma_D1 = 0.28992;
K_gamma_I1 = 3.9584;


p0 = 0.0095;
PO = 0.005;
Time = 0.35;
epsil = -log(PO) / sqrt(pi*pi + (log(PO))^2); %0.69
wn = 4 / (epsil*Time);

%K_gamma_D1 = (2*epsil * wn + p0) * I_x;
%K_gamma_P1 = (wn*wn + 2*epsil*wn*p0) * I_x;
%K_gamma_I1 = wn*wn*p0 * I_x;

K_gamma_D1 = 0.1963;
  K_gamma_P1 = 0.8343;
  K_gamma_I1 = 1.1438;

%K_gamma_D1 =0.4983
%    K_gamma_P1 = 4.7905
%   K_gamma_I1 = 10.2944

%K_gamma_D1 =1.5553
% K_gamma_P1 =  42.3140
% K_gamma_I1 = 113.3519


  K_gamma_D1 =  1.5251
   K_gamma_P1 = 39.2638
   K_gamma_I1 =  37.7538

K_theta_P1 = 1.85548;
K_theta_D1 = 0.28992;
K_theta_I1 = 3.9584;

K_psi_P1 = 3.11;
K_psi_D1 = 0.4858;
K_psi_I1 = 6.6322;


%% запуск модели и вывод графиков
quadReduced
out = sim('quadReduced');

% out.angles - Углы по кватернионам
% out.angles_PD - по ПД

figure('Name', 'Angles')
subplot(311)
hold on, grid on
plot(out.angles.time, out.angles.signals.values(:,1));
plot(out.angles_PD.time, out.angles_PD.signals.values(:,1));
plot(out.angles.time, ones(1,length(out.angles.time))*task_angles(1))
legend('quat', 'pd', 'task')

subplot(312)
hold on, grid on
plot(out.angles.time, out.angles.signals.values(:,2));
plot(out.angles_PD.time, out.angles_PD.signals.values(:,2));
plot(out.angles.time, ones(1,length(out.angles.time))*task_angles(2))
legend('quat', 'pd', 'task')

subplot(313)
hold on, grid on
plot(out.angles.time, out.angles.signals.values(:,3));
plot(out.angles_PD.time, out.angles_PD.signals.values(:,3));
plot(out.angles.time, ones(1,length(out.angles.time))*task_angles(3))
legend('quat', 'pd', 'task')

% углы по кватернионам
gamma_quat = out.angles.signals.values(:,1);
theta_quat = out.angles.signals.values(:,2);
psi_quat = out.angles.signals.values(:,3);

% углы по ПД регулятору
gamma_PD = out.angles_PD.signals.values(:,1);
theta_PD = out.angles_PD.signals.values(:,2);
psi_PD = out.angles_PD.signals.values(:,3);


figure('Name', 'Angles errors quat-PD')
subplot(311)
hold on, grid on
plot(out.angles.time, gamma_quat - gamma_PD)
subplot(312)
hold on, grid on
plot(out.angles.time, theta_quat - theta_PD)
subplot(313)
hold on, grid on
plot(out.angles.time, psi_quat - psi_PD)


figure('Name', 'Positions')
subplot(311)
hold on, grid on
plot(out.positions_quat.time, out.positions_quat.signals.values(:,1))
plot(out.positions_quat.time, out.positions_PD.signals.values(:,1))
legend('quat', 'pd')
subplot(312)
hold on, grid on
plot(out.positions_quat.time, out.positions_quat.signals.values(:,2))
plot(out.positions_quat.time, out.positions_PD.signals.values(:,2))
legend('quat', 'pd')
subplot(313)
hold on, grid on
plot(out.positions_quat.time, out.positions_quat.signals.values(:,3))
plot(out.positions_quat.time, out.positions_PD.signals.values(:,3))
legend('quat', 'pd')

figure('Name', 'Torques')
subplot(221)
hold on, grid on
plot(out.torques_quat.time, out.torques_quat.signals.values(:,1))
plot(out.torques_PD.time, out.torques_PD.signals.values(:,1))
legend('quat', 'pd')

title('Altitude')
subplot(222)
hold on, grid on
plot(out.torques_quat.time, out.torques_quat.signals.values(:,2))
plot(out.torques_PD.time, out.torques_PD.signals.values(:,2))
legend('quat', 'pd')

title('Angle')
subplot(223)
hold on, grid on
plot(out.torques_quat.time, out.torques_quat.signals.values(:,3))
plot(out.torques_PD.time, out.torques_PD.signals.values(:,3))
legend('quat', 'pd')

title('Angle')
subplot(224)
hold on, grid on
plot(out.torques_quat.time, out.torques_quat.signals.values(:,4))
plot(out.torques_PD.time, out.torques_PD.signals.values(:,4))
legend('quat', 'pd')
title('Angle')

figure('Name', 'Energy')
subplot(411)
hold on, grid on
plot(out.energy_angles_quat.time, out.energy_angles_quat.signals.values(:,1))
plot(out.energy_angles_quat.time, out.energy_angles_PD.signals.values(:,1))
legend('quat', 'pd')
subplot(412)
hold on, grid on
plot(out.energy_angles_quat.time, out.energy_angles_quat.signals.values(:,2))
plot(out.energy_angles_quat.time, out.energy_angles_PD.signals.values(:,2))
legend('quat', 'pd')
subplot(413)
hold on, grid on
plot(out.energy_angles_quat.time, out.energy_angles_quat.signals.values(:,3))
plot(out.energy_angles_quat.time, out.energy_angles_PD.signals.values(:,3))
legend('quat', 'pd')
subplot(414)
hold on, grid on
plot(out.energy_total_quat.time, out.energy_total_quat.signals.values(:,1))
plot(out.energy_total_quat.time, out.energy_total_PD.signals.values(:,1))
legend('quat', 'pd')

%%
% figure
% pt = [0 0 0];
% dir = [1 0 0 1];
% h = quiver3(pt(1),pt(2),pt(3), dir(1),dir(2),dir(3));
% xlim([-1 1])
% ylim([-1 1])
% zlim([-1 1])
% 
% for i = 1:length(gamma)
%   for j = 1:length(theta)
%     for k = 1:length(psi)
%       xfm = makehgtform('xrotate', gamma(i), 'yrotate', theta(j), 'zrotate', psi(k));
%       newdir = xfm * dir';
%       h.UData = newdir(1);
%       h.VData = newdir(2);
%       h.WData = newdir(3);
%       drawnow
%     end
%   end
% end
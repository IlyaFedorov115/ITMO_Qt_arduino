clear all
close all

% входы
y_d = 100; % высота (задающее воздействие по оси 'OY')
gamma_d = pi/3;
theta_d = pi/4;
psi_d = pi/5;

% физические параметры
l = 0.225; % расстояние от центра масс до двигателей
tau_y = 10; % сила прикладываемая к вертикальному движению
g = 9.81; % ускорение свободного падения
m = 1.44 ; % масса квадрокоптера
A_x = 0.25; % аэродинамический коэффициент вдоль оси 'OX'
A_y = 0.25; % аэродинамический коэффициент вдоль оси 'OY'
A_z = 0.25; % аэродинамический коэффициент вдоль оси 'OZ'

omega_r = 1; % внешний сигнал, который подадим чуть позже

%% для угловых скоростей
k = 1.045 * 10^(-11);
b = 1.14 * 10^(-7);
a_r = 1.14 * 10^(-7); % постоянная вращательного движения
m_s = [1/4 0 -1/(2*l) -1/(4*a_r); 1/4 -1/(2*l) 0 1/(4*a_r); 1/4 0 1/(2*l) -1/(4*a_r); 1/4 1/(2*l) 0 1/(4*a_r)];
%% параметры регуляторов для модели с кватернионом

K_y_D = 13.58; % коэффициент дифференцирующей составляющей общей тяги
K_y_P = 33.18; % коэффициент пропорциональной составляющей общей тяги

P_q1 = 20; % коэффициент пропорциональной составляющей для мнимого единичного вектора i
P_q2 = 20; % коэффициент пропорциональной составляющей для мнимого единичного вектора j
P_q3 = 20; % коэффициент пропорциональной составляющей для мнимого единичного вектора k
P_q = diag([P_q1 P_q2 P_q3]);

P_omega_x = 4; % коэффициент пропорциональной составляющей скорости вращения в доль оси 'OX'
P_omega_y = 4; % коэффициент пропорциональной составляющей скорости вращения в доль оси 'OY'
P_omega_z = 4; % коэффициент пропорциональной составляющей скорости вращения в доль оси 'OZ'
P_omega = diag([P_omega_x P_omega_y P_omega_z]);

Ix = 0.0151;
Iy = 0.0151;
Iz = 0.253;
Icm = diag([Ix Iy Iz]);

dt = 1e-4;

initial_angles = [0, 0, 0];
initial_quaternion = quaternion(initial_angles, 'euler', 'ZYX', 'frame');

task_angles = [gamma_d, psi_d, theta_d];
task_quaternion = quaternion(task_angles, 'euler', 'ZYX', 'frame');

%% параметры регуляторов для модели с ПД-регулятором
I_x = 0.0151; % момент инерции квадрокоптера при его вращении вдоль оси 'OX' 
I_y = 0.253; % момент инерции квадрокоптера при его вращении вдоль оси 'OY' 
I_z = 0.0151; % момент инерции квадрокоптера при его вращении вдоль оси 'OZ' 
I_r = 5.38 * 10 ^ (-5); % инерция ротора двигателя

K_gamma_D = 50;  % коэффициент дифференцирующей составляющей вращения вдоль оси 'OX' 10.4 / 36
K_gamma_P = 108; % коэффициент пропорциональной составляющей вращения вдоль оси 'OX' 22 / 80

K_theta_D = 58;   % коэффициент дифференцирующей составляющей вращения вдоль оси 'OZ' 11.8
K_theta_P = 141;  % коэффициент пропорциональной составляющей вращения вдоль оси 'OZ' 24

K_psi_D = 40;   % коэффициент дифференцирующей составляющей вращения вдоль оси 'OY' 25
K_psi_P = 143;  % коэффициент пропорциональной составляющей вращения вдоль оси 'OY' 95

%% запуск модели и вывод графиков
quad
out = sim('quad');

figure('Name', 'Angles')
subplot(311)
hold on, grid on
plot(out.angles.time, out.angles.signals.values(:,1));
plot(out.angles_PD.time, out.angles_PD.signals.values(:,1));
plot(out.angles.time, ones(1,length(out.angles.time))*task_angles(1))
subplot(312)
hold on, grid on
plot(out.angles.time, out.angles.signals.values(:,2));
plot(out.angles_PD.time, out.angles_PD.signals.values(:,2));
plot(out.angles.time, ones(1,length(out.angles.time))*task_angles(2))
subplot(313)
hold on, grid on
plot(out.angles.time, out.angles.signals.values(:,3));
plot(out.angles_PD.time, out.angles_PD.signals.values(:,3));
plot(out.angles.time, ones(1,length(out.angles.time))*task_angles(3))

gamma_quat = out.angles.signals.values(:,1);
theta_quat = out.angles.signals.values(:,2);
psi_quat = out.angles.signals.values(:,3);

gamma_PD = out.angles_PD.signals.values(:,1);
theta_PD = out.angles_PD.signals.values(:,2);
psi_PD = out.angles_PD.signals.values(:,3);

figure('Name', 'Angles errors')
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
subplot(312)
hold on, grid on
plot(out.positions_quat.time, out.positions_quat.signals.values(:,2))
plot(out.positions_quat.time, out.positions_PD.signals.values(:,2))
subplot(313)
hold on, grid on
plot(out.positions_quat.time, out.positions_quat.signals.values(:,3))
plot(out.positions_quat.time, out.positions_PD.signals.values(:,3))

figure('Name', 'Torques')
subplot(221)
hold on, grid on
plot(out.torques_quat.time, out.torques_quat.signals.values(:,1))
plot(out.torques_PD.time, out.torques_PD.signals.values(:,1))
title('Altitude')
subplot(222)
hold on, grid on
plot(out.torques_quat.time, out.torques_quat.signals.values(:,2))
plot(out.torques_PD.time, out.torques_PD.signals.values(:,2))
title('Angle')
subplot(223)
hold on, grid on
plot(out.torques_quat.time, out.torques_quat.signals.values(:,3))
plot(out.torques_PD.time, out.torques_PD.signals.values(:,3))
title('Angle')
subplot(224)
hold on, grid on
plot(out.torques_quat.time, out.torques_quat.signals.values(:,4))
plot(out.torques_PD.time, out.torques_PD.signals.values(:,4))
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
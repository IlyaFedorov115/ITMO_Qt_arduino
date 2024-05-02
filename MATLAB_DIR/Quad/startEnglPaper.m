%% очистка
clc, clear, close all;

%% Общие параметры
% физические параметры
l = 0.225; % расстояние от центра масс до двигателей

g = 9.81; 
m = 0.486 ;                  % масса квадрокоптера
A_x = 0.25;                  % аэродинамический коэффициент вдоль оси 'OX' кг/с
A_y = 0.25;                  % аэродинамический коэффициент вдоль оси 'OY'
A_z = 0.25;                  % аэродинамический коэффициент вдоль оси 'OZ'
kl = 3 * 10^(-6);
b = 1.14 * 10^(-7);
l = 0.225;

a_r = 0.033;                % постоянная вращательного движения
I_x = 4.856 * 10^(-3);      % момент инерции квадрокоптера при его вращении вдоль оси 'OX' 
I_y =  4.856 * 10^(-3);     % момент инерции квадрокоптера при его вращении вдоль оси 'OY' 
I_z =  8.801 * 10^(-3);     % момент инерции квадрокоптера при его вращении вдоль оси 'OZ' 
I_r = 3.38 * 10^(-5);       % инерция ротора двигателя

% -------- Статья + модель из ГИТ
k = 2.980e-6;        
Im = 3.357e-5;              % Inertia moment of the rotor
I_xx = 4.856e-3;            % kg*m^2
I_yy = 4.856e-3;            % kg*m^2
I_zz = 8.801e-3;            % kg*m^2

%% Константы мои
phi_init = 0;   % deg2rad(10);
theta_init = 0; % deg2rad(10);
psi_init = 0;  %deg2rad(10);
x_init = 0;
y_init = 0;

z_init = 0;
z_d = 0.2;

phi_d = 0.2;
theta_d = -0.1;
psi_d = 0.1;

K_z_D1 = 4.42; %  
K_z_P1 = 11.2; % 

K_phi_D1 = 0.047;% 1.75;   %  -------------- Вариант С без умножения на  tau CHANGE_PID
K_phi_P1 = 0.11;%6;
K_phi_I1 = 0;

K_theta_D1 = 0.047;% 1.75;
K_theta_P1 = 0.11;%6;
K_theta_I1 = 0;

K_psi_D1 = 0.0844;%1.75;
K_psi_P1 = 0.202;%6;
K_psi_I1 = 0;




K_z_D2 = 4.42/m + A_z/m; %  2.5 статья, 4.42 - ПД по пф,
K_z_P2 = 11.2/m + A_z/m; %  1.5 статья, 11.2 - ПД по пф,

K_phi_D2 = 9.6;     % 1.75; статья, ПД = ()*tau  | 9.6  ПД по пф
K_phi_P2 = 23.04;   % 6;    статья  | 23.04;   ПД по пф
K_phi_I2 = 0;

K_theta_D2 = 9.6;% 1.75;
K_theta_P2 = 23.04;%6;
K_theta_I2 = 0;

K_psi_D2 = 9.6;%1.75;
K_psi_P2 = 23.04;%6;
K_psi_I2 = 0;

%% Англ мои ПИД
K_phi_D22 = 19.2;    K_theta_D22 = 19.2;    K_psi_D2 = 19.2; 
K_phi_P22 = 122.88;  K_theta_P22 = 122.88;  K_psi_P2 = 122.88;
K_phi_I22 = 262.144; K_theta_I22 = 262.144; K_psi_I2 = 262.144;

K_phi_D11 = 0.093;    K_theta_D11 = 0.093;    K_psi_D11 = 0.169; 
K_phi_P11 = 0.5966;   K_theta_P11 = 0.5966;   K_psi_P11 = 1.081;
K_phi_I11 = 1.27;     K_theta_I11 = 1.27;     K_psi_I11 = 2.307;


%% 
% all angles PD [6, 1.75] - по статье англ. 
% all angles PD [23.04, 9.6] - посчитанный ПД для ПФ руками
% [0.11; 0.047] and [0.202, 0.0844] - для версии без умножения выхода ПД Ix


%% Константы для модели ГИТ
roll_i = deg2rad(10);   % rad
pitch_i = deg2rad(10);  % rad
yaw_i = deg2rad(10);      % rad
altitude_i = 1;  % meters

roll_d = 0;   % rad
pitch_d = 0;     % rad
yaw_d = 0;     % rad
altitude_d = 0;  % meters


% PD
K_z_D = 4.42;% 2.5; %  
K_z_P = 11.2;%1.5; % 

K_phi_D = 1.75;  %'OX' 10.4 / 36
K_phi_P = 6; %  'OX' 22 / 80

K_theta_D = 1.75;   %  'OZ' 11.8
K_theta_P = 6;  %  'OZ' 24

K_psi_D = 1.75;   %  'OY' 25
K_psi_P = 6;  %  'OY' 95

K_phi_D = 9.6;  %'OX' 10.4 / 36
K_phi_P = 23.04;%  'OX' 22 / 80

K_theta_D = 9.6;   %  'OZ' 11.8
K_theta_P = 23.04;  %  'OZ' 24

K_psi_D = 9.6;   %  'OY' 25
K_psi_P = 23.04;  %  'OY' 95




%% Модель по СТ, без умножения на Ixy и разными a_r
a_rS = 3.3 * 10^(-2); %-2 in ST %3.3 * 10^(-5);
mS = 1.44;
IxS = 0.0151;
IyS = IxS;
IzS = 0.0253;
IrS = 5.38 * 10^(-5);

K_z_PS = 33.18;
K_z_DS = 13.58;

K_phi_PS = 0.35;    % phi = gamma
K_phi_DS = 0.14;
K_theta_PS = 0.35;
K_theta_DS = 0.14;
K_psi_PS = 0.58;
K_psi_DS = 0.24;

x_iS = 0;
y_iS = 0;
z_iS = 1;

phi_iS = deg2rad(10);
theta_iS = deg2rad(10);
psi_iS = deg2rad(10);

z_DesS = 0.0;
phi_DesS = 0.2 * 0;
theta_DesS = -0.1 * 0;
psi_DesS = 0.1 * 0;

%%
% phi/theta PD [0.35, 0.14] - Посчитанные по ПФ для ПД с Ix, Iy, Iz из статьи СТ
% psi 0.58 0.24

%


%%
b = a_r;

%%
a_rS = b;


% По поводу коэффициента a_r и b. Нужно проверять, что везде его поменяли, он в двух местах. 
% Его изменение не влияет почему то на результат. Получается так, что чем он больше, 
% тем меньше разброс между w1,w2,w3,w4 в начале, и наоборот, поэтому это невелируется.

% Скорее всего это ПОКА, т.к. идет просто перевод из тау в омега и обратно.
% Когда добавится сатурация, тогда скорее всего будет уже влиять.
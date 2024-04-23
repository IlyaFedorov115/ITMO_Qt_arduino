
g = 9.81; % ускорение свободного падения
m = 1.44 ; % масса квадрокоптера
A_x = 0.25; % аэродинамический коэффициент вдоль оси 'OX'
A_y = 0.25; % аэродинамический коэффициент вдоль оси 'OY'
A_z = 0.25; % аэродинамический коэффициент вдоль оси 'OZ'


%% как в старт
Ix = 0.0151;
Iy = 0.0151;
Iz = 0.253;


%% как в статье
Ix = 0.0151;
Iz = 0.0151;
Iy = 0.253;


%% передаточные линеаризованной модели
Wy_s_num = [m^-1];
Wy_s_den = [1 A_y*m^-1 0]; % m^-1/(s(s+Aym^-1)

W_gamma_s_num = [Ix^-1];
W_gamma_s_den = [1 0 0];

W_theta_s_num = [Iz^-1];
W_theta_s_den = [1 0 0];

W_psi_s_num = [Iy^-1];
W_psi_s_den = [1 0 0];


% формула (11)
Wy_s_tf = tf(Wy_s_num, Wy_s_den)                % передаточная функция движения по y
W_gamma_tf = tf(W_gamma_s_num, W_gamma_s_den)   % передаточная функция гамма
W_theta_tf = tf(W_theta_s_num, W_theta_s_den)   % передаточная функция тета
W_psi_tf = tf(W_psi_s_num, W_psi_s_den)         % передаточная функция пси (рысканье)

%% передаточные в символьный вид
syms s;
syms kd_s;
syms kp_s;


symb_Wy_s_tf = poly2sym(Wy_s_tf.Numerator,s)/poly2sym(Wy_s_tf.Denominator,s) 
symb_W_gamma_tf = poly2sym(W_gamma_tf.Numerator,s)/poly2sym(W_gamma_tf.Denominator,s) 
symb_W_theta_tf = poly2sym(W_theta_tf.Numerator,s)/poly2sym(W_theta_tf.Denominator,s) 
symb_W_psi_tf = poly2sym(W_psi_tf.Numerator,s)/poly2sym(W_psi_tf.Denominator,s) 

%% ПД регулятор



% s^2 + 2w0s + wo^2
% t_targ = 1 sec
w_0_Newton2 = 4.8;

PD_tf = (kd_s*s + kp_s);
disp("PD TF: ")
disp(PD_tf);

disp("Wy+PD");
Wy_PD_tf = PD_tf * symb_Wy_s_tf / (1 + PD_tf * symb_Wy_s_tf)
disp("W_angle + PD");
W_gamma_PD = PD_tf*symb_W_gamma_tf / (1 + PD_tf*symb_W_gamma_tf)
W_theta_PD = PD_tf*symb_W_theta_tf / (1 + PD_tf*symb_W_theta_tf)
W_psi_PD = PD_tf*symb_W_psi_tf / (1 + PD_tf*symb_W_psi_tf)



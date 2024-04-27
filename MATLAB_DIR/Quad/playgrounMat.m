
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



%% Пид по статье https://elib.sfu-kras.ru/bitstream/handle/2311/20139/06_Prokopiev.pdf?sequence=1&ysclid=lvfbxtgg5t545236805
a0 = 0.04;
a1 = 0.12;
a2 = 1;
b0 = 0.01;
b1 = 0.7;
nu1 = 1;
nu2 = 5;
betta = 0.5;

l11 = b1 - b0*(nu1 + 2*nu2);
l12 = b0;
l13 = 0;
l21 = -b0*(betta^2 + nu2^2 + 2*nu1*nu2);
l22 = b1;
l23 = b0;
l31 = -b0*nu1*(betta^2 + nu2^2);
l32 = 0;
l33 = b1;

c1 = a0*(nu1 + 2*nu2)-a1;
c2 = a0 * (betta^2 + nu2^2 + 2*nu1*nu2)-a2;
c3 = a0 * nu1*(betta^2 + nu2^2);

L = [l11 l12 l13; l21 l22 l23; l31 l32 l33];
C = [c1; c2; c3];


%% попытка для нашей
a0 = I_x;
a1 = 0;
a2 = 0;
b0 = 0.00;
b1 = 1;
nu1 = 5;
nu2 = 100;
betta = 1.1;

l11 = b1 - b0*(nu1 + 2*nu2);
l12 = b0;
l13 = 0;
l21 = -b0*(betta^2 + nu2^2 + 2*nu1*nu2);
l22 = b1;
l23 = b0;
l31 = -b0*nu1*(betta^2 + nu2^2);
l32 = 0;
l33 = b1;

c1 = a0*(nu1 + 2*nu2)-a1;
c2 = a0 * (betta^2 + nu2^2 + 2*nu1*nu2)-a2;
c3 = a0 * nu1*(betta^2 + nu2^2);

L = [l11 l12 l13; l21 l22 l23; l31 l32 l33];
C = [c1; c2; c3];

K = inv(L)*C;
  K_gamma_D1 =  K(1)
   K_gamma_P1 = K(2)
   K_gamma_I1 = K(3)


%% Нули и полюса


Tf_gamma = tf([0.2899 1.8554 3.9584], [1 19.2 122.88 262.144]) 
pole(Tf_gamma)
zero(Tf_gamma)

p = [1 19.2 122.88 262.144];
q = [0.2899 1.8554 3.9584];
[quotient remainder] = deconv(p, q)

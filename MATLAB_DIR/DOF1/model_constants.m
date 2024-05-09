

SEP_STR = "====================================================";
CHAPTER_STR_START = ">>> ";
CHAPTER_STR_END = " <<<";
print_chapter = @(chap) fprintf("\n%s%s%s\n",CHAPTER_STR_START, chap, CHAPTER_STR_END);
print_sep = @() fprintf("\n%s\n", SEP_STR);

%%% =========== Model parameter ========== %%%
print_sep();
print_chapter("ПАРАМЕТРЫ МОДЕЛИ SIMULINK");

Sample_Time = 0.025 * 2;

PID_P_my = 0.25*0.6 * 0.7;
PID_I_my = (2*0.25*0.6)/1 * 0.7;
PID_D_my = 0.25*0.6*1/8 * 0.8;

PID_P_main = 0.3;% * 0.6;
PID_I_main = PID_P_main*2 / 0.7  * 0;
PID_D_main = PID_P_main * 0.7 / 8  * 0;

fprintf("\n%s\n", " *Константы ПИД*");
fprintf("\n  ПИД модели: %f, %f, %f", PID_P_my, PID_I_my, PID_D_my);
fprintf("\n  ПИД альтерантивный: %f, %f, %f", PID_P_main, PID_I_main, PID_D_main);

g = 9.806;
L_rod = 0.42; %m
m_mot = 0.06; %kg
m_esc = 0.025;
m_ardu = 0.015;
m_wood = 0.054;
m_rod = m_esc + m_ardu + m_wood;

fprintf("\n\n%s\n", " *Константы стенда*");

fprintf("\n  L_стерж = %f  Масса_стрежня = %f  Масса_мотора = %f", L_rod, m_mot, m_rod);

L_g = (m_mot*L_rod + m_rod*L_rod/2)/(m_rod + m_mot); % center mass
J_simple = (m_mot + m_rod)*L_rod^2;    % inertia moment
J_g = m_mot*L_rod^2 + (m_rod*L_rod^2)/3;
J = J_g;


fprintf("\n  L_цм = %f   J_цм = J_sim = %f  J_простой = %f", L_g, J_g, J_simple);


% Params of BLDC indentif-ed
K_bldc = 131.3016804;                         % rad/Vs
T_bldc = 0.010214509;
Kv_bldc = K_bldc;                       % rad/Vs
Ke_bldc = 7.616048 * 10^(-3);           %Vs/rad 
Kt_bldc = Ke_bldc;                      % Nm/A
Jmot_bldc = 4.23203*10^(-6);
R_bldc = 0.09;                          % study and sites 90 mO

fprintf("\n\n%s\n", " *Константы двигателя*");

fprintf("\n  K_bldc = %f rad/Vs  T_bldc = %f\n  Kv_bldc = K_bldc,  Ke_bldc = %f rad/Vs Kt_bldc = Ke_bldc Nm/A", ...
    K_bldc, T_bldc, Ke_bldc);

% F = KuL -> K*PWM*L -> K
% K*PWM*L = (M+m)*Lg*g
K_test = (m_rod + m_mot)*L_g*g/(1443*L_rod);

% Params and constants from formulas
r_r = 0.095;        %m [20] radius of propeller
rho_air = 1.204;    %kg/m3
w_ss = 437;         %rad/s - omega rotor for pwm=1443=pwm_ss
u_ss = 3.2;         % V
Tg_ss = (m_mot + m_rod)*L_g*g;
Fg_ss = (m_mot + m_rod)*g;
C_T = Tg_ss / (r_r^4 * rho_air * pi * w_ss^2 * L_rod);


K_f1 = r_r^4 * rho_air * pi * C_T;
K_f2 = K_f1 * w_ss^2 / u_ss;
K_f3 = K_f1 * w_ss^2 / 1440;    % == (m_rod + m_mot) * g * L_g / (1440*L_rod)
K_f33 = (m_rod + m_mot) * g * L_g / ((1440-1200)*L_rod);   % 1200 - 2000 -> 0 - 800 + 1200
fprintf("\n  Пропеллер_r = %f  ρ_возд. = %f  ω_ss ≈ %f", r_r, rho_air, w_ss);
fprintf("\n  𝜏_mot = Tg_ss = k1 * ω^2 * L = k2 * u * L = k3 * PWM * L");

fprintf("\n\n%s\n", " *Константы для мат. модели*");

fprintf("\n  Установившийся режим: Tmot = Tg_ss = %f, Fg_ss = %f", Tg_ss, Fg_ss);

fprintf("\n  k1 = r^4*ρ*π*C_T -> C_T = Tg_ss/L(w_ss^2*k1_part) = %f  ", C_T);
fprintf("\n  k2 = k1*ω_ss^2/u_ss, sin(θss) = k2*uss/mg -> k2 = mg/uss * (L_g/L_rod) - равны")
fprintf("\n  k1 = %f k2 = %f, k3 = %f, k33(0-800) = %f", K_f1, K_f2, K_f3, K_f33);


r_r^4 * rho_air * pi * w_ss^2 * C_T * L_rod
Tg_ss








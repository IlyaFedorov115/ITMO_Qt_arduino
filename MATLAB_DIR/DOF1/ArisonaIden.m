%% константы
m_all = 0.3;
L_rod = 0.57;
d_cm = 0.33;
J = 0.048;
g = 9.81;
c_visc = 5*10^(-5);
%% данные

% -- может со старым проблема была в том, что настраивали с 1000 до 2000, а
% надо было с 1250 до 2000??

u_pwm = [1358 1370 1374 1378 1384 1392 1398 1402 1408 1414 1422 1428 1434 1440 1444 1448 1452 1456];
u_pwm255_AllScale = rescale(u_pwm, 0, 255, "InputMin", 1000, "InputMax", 2000); % для нового все равно, т.к. с 1080
u_pwm_1000 = u_pwm - 1000;
err_arr = [45 42 41 40 38.5 37.2 35.1 33.3 31.5 29.8 27.5 26.0 24.35 21.45 17.65 16.2 12.9 8.0];
theta_arr = 90 - err_arr;
theta_arr_rad = deg2rad(theta_arr);
theta_sin_ss = sin(theta_arr_rad); %% sind

%% Выбор параметра
u_ss = u_pwm255_AllScale;


%% 
figure
plot(theta_sin_ss, u_ss, '-+');
ylabel('PWM input');
xlabel('sin theta_s');
grid on;

%% Определение зависимости
sine_4_iden = theta_sin_ss(1:end);
u_4_iden = u_ss(1:end);

P_iden = polyfit(sine_4_iden, u_4_iden, 1);
figure
plot(theta_sin_ss, u_ss, '-+', sine_4_iden, polyval(P_iden, sine_4_iden), '-red', 'LineWidth',1)
legend('Experimental', 'Linear fit')
grid on;
fprintf('\nПараметры: ');
disp(P_iden);
fprintf('\nK bldc iden: %f ', m_all*g*d_cm/(L_rod*P_iden(1)));
fprintf('\nK bldc iden simpl: %f ', m_all*g/P_iden(1));
fprintf('\nStart pwm: %f\n', rescale(P_iden(2), 1000, 2000, "InputMin", 0, "InputMax", 255))


%% Результаты для отклика на ступенчатый step4
startFrom = 600 + 34;
uStep = rescale(arr_uSignal1300, 0, 255, "InputMin", 1000, "InputMax", 2000);
uStep = uStep(startFrom:end);
yAns = arr_angleFilt(startFrom:end);
yPlot = arr_angle(startFrom:end);

%yAns = deg2rad(yAns);
%yPlot = deg2rad(yPlot);


tAll = arr_timestamps(startFrom:end) - arr_timestamps(startFrom);


DAT = iddata(yAns,uStep,mean(diff(arr_timestamps)));

m1 = pem(DAT,'p2u')
%m1 = pem(DAT,2)
% Convert the model to state space description CL
CL=idss(m1);
% Convert state space description to transfer function
[n1 d1]=ss2tf(CL.A,CL.B,CL.C,CL.D,1)
% Ingore n1(1) and n1(2) due to being very small
gCL=tf(n1(3),d1)
%gCL = W_model
% Simulate response from the model and compare with experiment
[y1,t1]=lsim(gCL,uStep,tAll);
figure
%plot(tAll,arr_angle(startFrom:end),t1,y1)
plot(tAll,yPlot, 'LineWidth', 1)
hold on
plot(t1,y1, 'LineWidth', 1)

y_value = mean(yPlot(end-100:end)); % Значение по оси Y, где будет проходить горизонтальная линия
line_color = 'red'; % Цвет линии (красный)
line_style = '--'; % Стиль линии (пунктирный)
line_width = 1; % Толщина линии
xticks(0:1:max(tAll)); % Устанавливаем отметки с шагом 1 секунда от 0 до 10
yticks(0:5:max(yPlot)); % Устанавливаем отметки с шагом 1 секунда от 0 до 10

yline(y_value, line_style, 'Color', line_color, 'LineWidth', line_width);

xlabel('t (s)', 'FontSize', 12); % Подпись оси X
ylabel('angle (deg)', 'FontSize', 12); % Подпись оси Y

grid on;
legend('Experiment','Estimated','location','SouthEast')
title('Step Response')





%% константы
dir = fullfile('c:\','ITMO_2024_WORK/ITMO_Qt_arduino/MATLAB_DIR/DOF1/Logs_OneDofHelicopter/17_05_NEw_bldc/CSV/');
filename = 'Ступенчатый_область_70.csv'; % sine2_2 Ступенчатый_сигнал_from_0  Ступенчатый_область_70
filePath = fullfile(dir, filename);

%% чтение данных
data_table = readtable(filePath);
data_table
% error,errorFilt,angle,angleRad,uSignal255,uSignal255Int,uSignal1300,pwm_values,time_steps,timestamps

%% чтение в массив
arr_error = data_table.error;
arr_errorFilt = data_table.errorFilt;
arr_angle = data_table.angle;
arr_angleFilt = data_table.angleFilt;
arr_filtVelocity = data_table.filtVelocity;
arr_velocity = data_table.velocity;

arr_velocity = data_table.errorFilt;


arr_angleRad = data_table.angleRad;
arr_uSignal255 = data_table.uSignal255;
arr_uSignal255Int = data_table.uSignal255Int;
arr_uSignal1300 = data_table.uSignalPWM;
arr_pwm_values = data_table.pwm_values;
arr_time_steps = data_table.time_steps;
arr_timestamps = data_table.timestamps;

%% plot данные
figure;
plot(arr_timestamps, arr_uSignal1300);
title('U signal'); xlabel('Time');
legend('u'); grid on;

figure;plot(arr_timestamps, arr_angle);
grid on; xlabel('Time'); title('Angle');
legend('angle');

figure;plot(arr_timestamps, arr_angleFilt);
grid on; xlabel('Time'); title('filtAngle');
legend('angle');

figure;plot(arr_timestamps, arr_velocity);
grid on; xlabel('Time'); title('Velocity');
legend('angle');

figure;plot(arr_timestamps, arr_filtVelocity);
grid on; xlabel('Time'); title('Velocity filt');
legend('angle');
%% подготовка для МНК
every = 2;
colY = arr_angle;
colU = arr_uSignal255;
%colU = arr_uSignal255Int;
colU = arr_uSignal255 - rescale(1430, 0, 255, "InputMin", 1300, "InputMax", 2000);

colU = rescale(arr_pwm_values, 0, 255, "InputMin", 1000, "InputMax", 2000);
%colU = arr_uSignal1300;
colT = arr_timestamps;

colU = colU(1:every:end);
colY = colY(1:every:end);
colT = colT(1:every:end);

V = diff(colY)./mean(diff(colT));
dV = diff(V)./mean(diff(colT));%diff(colT(1:end-1));   % dV/dt
Q = [colU(1:end-2) V(1:end-1) colY(1:end-2)];
Qt = pinv(Q);

%% plot dV/dt 
figure;
subplot(2,1,1); plot(colT(1:end-1), V);  title('V');
subplot(2,1,2); plot(colT(1:end-2), dV); title('dV');
grid on;
%%
figure;
 plot(colT, colY); hold on; plot(colT, Y_filt); grid on;
	title('Filtering Y'); legend('Output', 'Filter out');
grid on;
%% filt V
filt_V_dv = 0;
if filt_V_dv == 1
% Filting data before diff
	freq = 1000 / mean(diff(colT)); % Герц
	freq = 0.00001;
	%freq = 1 / mean(diff(colT));
	%w_freq = 2 * pi * freq;
	%alpha_filt = 1 / w_freq;
	alpha_filt = 1 / freq;
    %alpha_filt = 0.01;
	tf_lp = tf(1, [alpha_filt 1]);
	t_for_filt = 0:alpha_filt:((size(colY)-1)*alpha_filt);
	Y_filt = lsim(tf_lp, colY', t_for_filt); 

    Y_filt = lowpass(colY, 0.005);

	%Y_filt = filter(1, [alpha_filt 1], colY);
	figure; plot(colY); hold on; plot(Y_filt); grid on;
	title('Filtering Y'); legend('Output', 'Filter out');
	
	% Filt V
    freq  = 5.01;
	alpha_filt = 1 / freq;
	t_for_filt = 0:alpha_filt:((size(V)-1)*alpha_filt);
    %tf_lp = tf(1, [alpha_filt 1]);
	V_filt = lsim(tf_lp, V', t_for_filt); 
	V_ = V_filt;

    V_ = lowpass(V, 0.5);
   % V_ = lowpass(V, 1, 1000);

	dV_ = diff(V_)./mean(diff(colT));%diff(colT(1:end-1));   % dV/dt
	t_for_filt = 0:alpha_filt:((size(dV_)-1)*alpha_filt);
	dV_filt = lsim(tf_lp, dV_', t_for_filt);
	
    dV_filt = lowpass(dV_, 0.5);

	figure; subplot(3,1,1);
	plot(colT(1:end-1), V_); title('V after filt'); grid on;
	subplot(3,1,2); plot(colT(1:end-2), dV_);  title('dV/dt after V filt');grid on;
	subplot(3,1,3);
	plot(colT(1:end-2), dV_filt);
	title('dV/dt after filt');grid on;	
else
	V_filt = V;
	dV_filt = dV;
    %Y_filt = colY;
end

%% parameters
%Y_filt = colY;
%Y_filt = arr_angleFilt(1:every:end);
Q = [colU(1:end-2) V_filt(1:end-1) Y_filt(1:end-2)];
%Q = [colU(1:end-2) V_filt(1:end-1) deg2rad(Y_filt(1:end-2))];
%Q = [colU(1:end-2) V_filt(1:end-1) ones(size(colU(1:end-2)))*0.3*9.81*0.84*0.33/0.05];
Qt = pinv(Q);
params = Qt * dV_filt;
b = params(1);
a1 = -params(2);
a0 = -params(3);
fprintf("\n>>> Parameters W: b = %f, a1 = %f, a0 = %f\n", b, a1, a0);

%% modeling with coefficients

%b = 0.02932*0.575/0.048;
%b = 0.02932*0.575/0.048;
%a1 = 5*10^(-2)/0.048;
%a0 = 0.3*0.33*9.81/0.048 ;

%alpha_filt = 0.025;
alpha_filt = mean(diff(colT));%mean(colT);
%alpha_filt = mean(colT);
W_model = tf(b*1., [1 a1*1.0 a0*1.00275]);
t_for_filt = 0:alpha_filt:((size(colU)-1)*alpha_filt);
Out_Model = lsim(W_model, colU, t_for_filt);

%Out_Model = rad2deg(Out_Model);

figure;
plot(colT, Out_Model);
hold on;
plot(colT, colY); %Y_filt
grid on;
legend('Transfer', 'Real system');

%% С использованием угловой скорости
V_n = arr_velocity;
Y_n = arr_angleFilt;
%V_n = arr_filtVelocity;
%Y_n = arr_angle;


%V_n = lowpass(V_n, 0.1);


V_n = V_n(1:every:end); Y_n = Y_n(1:every:end);
Q = [colU(1:end-1) V_n(1:end-1) Y_n(1:end-1)];
dV_n = diff(V_n)./mean(diff(colT));

%Q = [colU(1:end-2) V_filt(1:end-1) deg2rad(Y_filt(1:end-2))];
%Q = [colU(1:end-2) V_filt(1:end-1) ones(size(colU(1:end-2)))*0.3*9.81*0.84*0.33/0.05];
alpha_filt = 1 / 5;
%t_for_filt = 0:alpha_filt:((size(dV_n)-1)*alpha_filt);
%dV_filt_n = lsim(tf_lp, dV_n', t_for_filt);
dV_filt_n = lowpass(dV_n, 0.01);
%dV_n = dV_filt_n;
figure;
plot(colT(1:end-1), dV_filt_n);

Qt = pinv(Q);
params = Qt * dV_n;
b = params(1)*2;
a1 = -params(2)*1.2;
a0 = -params(3)*1.95;

%b = 06.047;
%a1 = 1.2873;
%a0 = 14.065;
fprintf("\n>>> Parameters W: b = %f, a1 = %f, a0 = %f\n", b, a1, a0);
%W_model
alpha_filt = mean(diff(colT));%mean(colT);
W_model = tf(b*1., [1 a1 a0]);
%W_model = gCL
t_for_filt = 0:alpha_filt:((size(colU)-1)*alpha_filt);
Out_Model = lsim(W_model, colU, t_for_filt);
Out_Model = lsim(W_model, colU, colT)*1.0;
figure;
plot(colT, Out_Model);
hold on;
plot(colT, colY); %Y_filt colY
grid on;

xlabel('t (s)', 'FontSize', 12); % Подпись оси X
ylabel('angle (deg)', 'FontSize', 12); % Подпись оси Y
xticks(0:5:max(colT)*1.1); % Устанавливаем отметки с шагом 1 секунда от 0 до 10
yticks(0:5:max(colY)); % Устанавливаем отметки с шагом 1 секунда от 0 до 10


legend('MHK', 'Experimental');

%% Система по отклику найденная
b = 5.964*1;
a1 = 1.278;
a0 = 14.08*1.00;
b=1.47; a1=1.146; a0=2.778;
t_for_filt = 0:alpha_filt:((size(colU)-1)*alpha_filt);

W_modelStepResp = tf(b, [1 a1 a0])
Out_ModelStepResp = lsim(W_modelStepResp, colU, colT)*1.0;


b = 1.908;
a1 = 0.7188;
a0 = 4.561;

W_MNK_Clear = tf(b, [1 a1 a0])
Out_ModelMNK_Clear = lsim(W_MNK_Clear, colU, colT)*1.0;


b = 1.908*3;
a1 = 0.7188;
a0 = 4.561*3;


W_MNK_Div = tf(b, [1 a1 a0])

Out_ModelMNK_Div = lsim(W_MNK_Div, colU, colT)*1.0;

b = (5.964+1.908)/2;
a1 = (1.278+0.7188)/2;
a0 = (14.08+4.561)/3;




W_MNK_mean = tf(b, [1 a1 a0])

Out_ModelMNK_mean = lsim(W_MNK_mean, colU, colT)*1.0;

b = 4.316*1.0;
a1 = 1.845;
a0 = 9.118*1.0;

b = 2.393622; a1 = 0.865914; a0 = 5.086699;
%b = 1.019680; a1 = 0.439367; a0 = 2.275356
W_MNK_IdentBlock = tf(b, [1 a1 a0])
%W_model_curr = W_MNK_mean;
Out_ModelIdent = lsim(W_MNK_IdentBlock, colU, colT)*1.0;


figure;
plot(colT, deg2rad(Out_ModelStepResp));
hold on;
plot(colT, deg2rad(Out_ModelMNK_Clear));
hold on;
%plot(colT, Out_ModelMNK_Div);
hold on;
%plot(colT, Out_ModelIdent);
%hold on;
plot(colT, deg2rad(colY)); %Y_filt
grid on;
legend('StepResp', 'MNK_Clear', 'MNK_ident','Real system');
legend('MNK', 'Code', 'Real system');
xlabel('t (s)', 'FontSize', 12); % Подпись оси X
ylabel('angle (rad)', 'FontSize', 12); % Подпись оси Y

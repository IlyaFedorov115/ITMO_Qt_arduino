%% константы
dir = fullfile('c:\','ITMO_2024_WORK/ITMO_Qt_arduino/MATLAB_DIR/DOF1/Logs_OneDofHelicopter/garmonics_and_steps/hard_zakryt/');
filename = 'using_gyro\new_step.csv';
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
arr_angleRad = data_table.angleRad;
arr_uSignal255 = data_table.uSignal255;
arr_uSignal255Int = data_table.uSignal255Int;
arr_uSignal1300 = data_table.uSignal1300;
arr_pwm_values = data_table.pwm_values;
arr_time_steps = data_table.time_steps;
arr_timestamps = data_table.timestamps;

%% plot данные
figure;
plot(arr_timestamps, arr_uSignal255);
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
every = 5;
colY = arr_angle;
colU = arr_uSignal255;
%colU = arr_uSignal255Int;
colU = arr_uSignal1300;
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
b = params(1);
a1 = -params(2);
a0 = -params(3);
fprintf("\n>>> Parameters W: b = %f, a1 = %f, a0 = %f\n", b, a1, a0);

alpha_filt = mean(diff(colT));%mean(colT);
W_model = tf(b*1., [1 a1*1.0 a0*1.00275]);
t_for_filt = 0:alpha_filt:((size(colU)-1)*alpha_filt);
Out_Model = lsim(W_model, colU, t_for_filt);
figure;
plot(colT, Out_Model);
hold on;
plot(colT, colY); %Y_filt
grid on;
legend('Transfer', 'Real system');
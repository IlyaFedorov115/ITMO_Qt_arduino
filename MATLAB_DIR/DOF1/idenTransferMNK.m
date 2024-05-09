%% constants
bool_plot_V_dv_before_filt = 1;
filt_V_dv = 0;
modeling_with_coefs = 0;

dir = fullfile('c:\','ITMO_2024_WORK/Master_ITMO_2024/','mat_scripts_identification/logs/');
filename = 'log_03-03-2024_11-50-02.csv';
filePath = fullfile(dir, filename);

%% supp functions

min_volt = 0;
max_volt = 11.1;
min_pwm = 1200;
max_pwm = 2000;
min_pwm_zero = 0;
max_pwm_zero = max_pwm - min_pwm;       % отсчет от нуля

volt2pwm = @(v) rescale(v, min_pwm, max_pwm, "InputMin", min_volt, "InputMax", max_volt);
pwm2volt = @(pwm) rescale(pwm, min_volt, max_volt, "InputMin", min_pwm, "InputMax", max_pwm);
normilize_pwm = @(pwm) rescale(pwm, min_pwm_zero, max_pwm_zero, "InputMin", min_pwm, "InputMax", max_pwm);


%% Read data

dataCsv = readmatrix(filePath);
colT = dataCsv(:, 3);
colSig = data(:, 1);
colAngle = data(:, 2);
colAngleSin = colAngle + 90;

%% plot signal and output
figure;
plot(colT, colSig / 1000 );
hold on;
plot(colT, colAngle);
grid on;
xlabel('Time');
title('PWM and angle');
legend('PWM', 'angle');

%% prepare to calc transfer
colY = colAngleSin;
colU = pwm2volt(colSig);

V = diff(colY)./mean(diff(colT));
dV = diff(V)./mean(diff(colT));%diff(colT(1:end-1));   % dV/dt
Q = [colU(1:end-2) V(1:end-1) colY(1:end-2)];
Qt = pinv(Q);

%% Plot Velocity and dV/dt
if bool_plot_V_dv_before_filt == 1
    figure;
    subplot(2,1,1);
    plot(colT(1:end-1), V);
    title('V');
    subplot(2,1,2);
    plot(colT(1:end-2), dV);
    title('dV');
    %hold on;
    grid on;
end

%% Filg V
if filt_V_dv == 1
% Filting data before diff
	freq = 1000 / mean(diff(colT)); % Герц
	freq = 5;
	freq = 1 / mean(diff(colT));
	w_freq = 2 * pi * freq;
	alpha_filt = 1 / w_freq;
	alpha_filt = 1 / freq;
	tf_lp = tf(1, [alpha_filt 1]);
	t_for_filt = 0:alpha_filt:((size(colY)-1)*alpha_filt);
	Y_filt = lsim(tf_lp, colY', t_for_filt); 
	%Y_filt = filter(1, [alpha_filt 1], colY);
	figure;
	plot(colY);
	hold on
	plot(Y_filt);
	grid on;
	title('Filtering Y');
	legend('Output', 'Filter out');
	
	% Filt V
	alpha_filt = 0.02;
	t_for_filt = 0:alpha_filt:((size(V)-1)*alpha_filt);
	V_filt = lsim(tf_lp, V', t_for_filt); 
	V = V_filt;
	dV = diff(V)./mean(diff(colT));%diff(colT(1:end-1));   % dV/dt
	t_for_filt = 0:alpha_filt:((size(dV)-1)*alpha_filt);
	dV_filt = lsim(tf_lp, dV', t_for_filt);
	
	figure;
	subplot(3,1,1);
	plot(colT(1:end-1), V);
	title('V after filt'); grid on;
	subplot(3,1,2);
	plot(colT(1:end-2), dV);
	title('dV/dt after V filt');grid on;
	subplot(3,1,3);
	plot(colT(1:end-2), dV_filt);
	title('dV/dt after filt');grid on;	
else
	V_filt = V;
	dV_filt = dV;
end



%% calc coefficients

Q = [colU(1:end-2) V_filt(1:end-1) Y_filt(1:end-2)];
Qt = pinv(Q);
params = Qt * dV_filt;
b = params(1);
a1 = -params(2);
a0 = -params(3);
fprintf("\n>>> Parameters W: b = %f, a1 = %f, a0 = %f\n", b, a1, a0);

%% modeling with coefficients

% alpha_filt = 0.025;
W_model = tf(b, [1 a1 a0]);
t_for_filt = 0:alpha_filt:((size(colU)-1)*alpha_filt);
Out_Model = lsim(W_model, colU, t_for_filt);
figure;
plot(t_for_filt, Out_Model);
hold on;
plot(colT, colY);
grid on;
legend('Transfer', 'Real system');

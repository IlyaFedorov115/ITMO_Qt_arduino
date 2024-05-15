z = iddata(arr_angleFilt(1:every:end), colU, mean(diff(colT)));
z = iddata(deg2rad(arr_angleFilt(1:every:end)), colU, mean(diff(colT)));

z.InputName = 'PWM';
z.OutputName = 'Angle';
figure;
plot(z(:,1,:))


%%
m = 0.3;
L = 0.57;
d = 0.33;
J = 0.048;
g = 9.81;
c = 5*10^(-5);
K = 0.029;
A = [0 1; -m*d*9.81*0.76/J -c/J];
A = [0 1; -m*d*9.81/J -c/J];

B = [0; K*L/J];
C = [1 0];
D = zeros(2,1);
D = 0;
%%
ms = idss(A,B,C,D);
ms.Structure.a.Free = [0 0; 0 1];
ms.Structure.b.Free = [0; 1];
ms.Structure.c.Free = 0; % scalar expansion used
ms.Structure.d.Free = 0;
ms.Ts = 0;  % This defines the model to be continuous
ms
%%
dcmodel = ssest(z,ms,ssestOptions('Display','on'));
dcmodel

%% ANOTHER
FileName      = 'Pend1';        % File describing the model structure.
Order         = [1 1 2];             % Model orders [ny nu nx].
Parameters    = [m; g; L; d; c; J; K];   % Initial parameters. u,m,g,L,d,c,J, K
InitialStates = [1; 0];              % Initial initial states.
Ts            = 0;                   % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts);
nlgr.OutputName = 'Pendulum position';
nlgr.OutputUnit = 'rad';
nlgr.TimeUnit = 's';
nlgr = setinit(nlgr, 'Name', {'Pendulum position' 'Pendulum velocity'});
nlgr = setinit(nlgr, 'Unit', {'rad' 'rad/s'});
%nlgr = setpar(nlgr, 'Name', {'Gravity constant' 'Length', 'Friction coefficient' 'Mass'});
%nlgr = setpar(nlgr, 'Unit', {'m/s^2' 'm' 'Nms/rad' 'kg'});
nlgr = setpar(nlgr, 'Minimum', {eps(0) eps(0) eps(0) eps(0) eps(0) eps(0) eps(0)});   % All parameters > 0.

%% 
% A. Model computed with first-order Euler forward ODE solver.
nlgref = nlgr;
nlgref.SimulationOptions.Solver = 'ode1';        % Euler forward.
nlgref.SimulationOptions.FixedStep = z.Ts*mean(diff(colT));   % Step size.

% B. Model computed with adaptive Runge-Kutta 23 ODE solver.
nlgrrk23 = nlgr;
nlgrrk23.SimulationOptions.Solver = 'ode23';     % Runge-Kutta 23.

% C. Model computed with adaptive Runge-Kutta 45 ODE solver.
nlgrrk45 = nlgr;
nlgrrk45.SimulationOptions.Solver = 'ode45';     % Runge-Kutta 45.

compare(z, nlgref, nlgrrk23, nlgrrk45, 1, ...
   compareOptions('InitialCondition', 'model'));

%% ----------------------
yrad = deg2rad(arr_angleFilt(1:every:end));
u = colU;
Ts = mean(diff(colT));

data = iddata(yrad, u, Ts, 'Name', 'Experiment');
data = detrend(data);
%data = data(800:end);
data.InputName = 'Voltage';
data.InputUnit = 'V';
data.OutputName = 'Pendulum Angle';
data.OutputUnit = 'rad';
data.Tstart = 0;
data.TimeUnit = 's';

theta0 = data.y(1,1);
dtheta0 = (data.y(2,1) - data.y(1,1))/Ts;
%% Initial guess of parameters
m = 0.31;
g = 9.81;
l = 0.33;
c = 0.001;
K = 0.01;
InitialStates = [theta0; dtheta0];
Ts = 0;
modelType = 'nonlinear';
%modelType = 'linear';

% validation
v_data = data;

%%
switch modelType
    case 'nonlinear'
        % Nonlinear greybox model identification
        FileName = "AeroShield_ODE";
        Order = [1, 1, 2];
        Parameters = {m, g, l, c, K};
        
        nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, 'Name', 'Nonlinear Grey-box Model');
        set(nlgr, 'InputName', 'Voltage', 'InputUnit', 'V', 'OutputName', 'Pendulum Angle', 'OutputUnit', 'rad', 'TimeUnit', 's');
        nlgr = setpar(nlgr, 'Name', {'Pendulum Mass', 'Gravitational Acceleration', 'Pendulum Length', 'Viscous Damping', 'Thrust'});
        nlgr = setinit(nlgr, 'Name',{'Pendulum Angle', 'Pendulum Angular Velocity'});
        
        nlgr.Parameters(1).Fixed = true;
        nlgr.Parameters(1).Minimum = 0.005;
        nlgr.Parameters(1).Maximum = inf;
        
        nlgr.Parameters(2).Fixed = true;
        nlgr.Parameters(2).Minimum = 0;
        nlgr.Parameters(2).Maximum = 10.0;
        
        nlgr.Parameters(3).Fixed = true;
        % nlgr.Parameters(3).Minimum = 0.001;
        % nlgr.Parameters(3).Maximum = inf;
        % 
        % nlgr.Parameters(4).Fixed = false;
        % nlgr.Parameters(4).Minimum = 0;
        % nlgr.Parameters(4).Maximum = inf;
        % 
        % nlgr.Parameters(5).Fixed = false;
        % nlgr.Parameters(5).Minimum = 0;
        % nlgr.Parameters(5).Maximum = inf;
        
        nlgr.InitialStates(1).Fixed = false;
        nlgr.InitialStates(2).Fixed = false;
        nlgr.SimulationOptions.Solver = 'ode45';
        
        size(nlgr)
        present(nlgr)
        opt = nlgreyestOptions('Display', 'on', 'EstCovar', true, 'SearchMethod', 'Auto');
        opt.SearchOption.MaxIter = 40;
        model = nlgreyest(data, nlgr, opt);
        
        %Linearization of the identified model
        m_ = model.Parameters(1).Value;
        g_ = model.Parameters(2).Value;
        l_ = model.Parameters(3).Value;
        c_ = model.Parameters(4).Value;
        K_ = model.Parameters(5).Value;
        syms dxdt y x1 x2 u
        dxdt=[x2;
            u*(K_/(l_*m_))-(g_/l_)*sin(x1)-x2*(c_/(m_*l_^2))];
        y=[x1;
            x2];
        
        Alin=jacobian(dxdt,[x1 x2]);
        Blin=jacobian(dxdt,u);
        Clin=jacobian(y,[x1 x2]);
        Dlin=jacobian(y,u);
        xlin=[pi/4 0];
        ulin=0;
        Ac=eval(subs(Alin,[x1 x2 u],[xlin ulin]));
        Bc=eval(subs(Blin,[x1 x2 u],[xlin ulin]));
        Cc=eval(subs(Clin,[x1 x2],xlin));
        Dc=eval(subs(Dlin,[x1 x2],xlin));
        linsys=ss(Ac,Bc,Cc,Dc);
        %save AeroShield_GreyboxModel_Nonlinear.mat model
        %save AeroShield_GreyboxModel_Linear.mat linsys
        
        %Model Validation
        compare(v_data,model,linsys)

    case 'linear'
        A = [0 1;
            -g/l -c/(m*l^2)];
        B = [0;
            K/(l*m)];
        
        C = [1 0];
        
        D = [0];
        K = zeros(2,1);
        
        disp('Initial guess:')
        sys = idss(A,B,C,0,K,InitialStates,0)
        sys.Structure.A.Free = [0 0;
                                1 1];
        sys.Structure.B.Free = [0;
                                1];
        sys.Structure.C.Free = false;
        sys.DisturbanceModel = 'estimate';
        %sys.InitialState = 'estimate';
        
        Options = ssestOptions;
        Options.Display = 'on';                    
        Options.Focus = 'prediction';              % Focus on simulation
        Options.SearchMethod = 'auto';
        Options.SearchOptions.MaxIterations = 1000;  % Max iterations
        Options.SearchOptions.Tolerance = 0.0000001;  % Tolerance
        Options.InitialState = 'estimate';         % Estimate initial condition
        disp('Estimated model:')                       
        linsys = ssest(data,sys,Options);           % Identified Model
        
        lest = -g/linsys.A(2,1) %Calculation of parameters
        cest = -linsys.A(2,2)*lest*m
        Kest = linsys.B(2,1)*lest*m
        compare(v_data,linsys)
        %save AeroShield_GreyboxModel_Linear.mat linsys
end

%%
compare(v_data,linsys)
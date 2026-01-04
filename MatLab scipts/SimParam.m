clear;
act.max_stroke = 0.200;
p.g = 9.81;     % [m/s^2] gravitational acceleration


% ------ Test variables -----
p.M = 150;      % [kg]
p.Ix = 20;      % [kgm^2] (roll inertia)
p.Iy = 25;      % [kgm^2] (pitch inertia)

g.t = 0.1;      % [m]   Motion platform thickness
g.dx = 0.5;     % [m]
g.dy = 0.2;     % [m]
g.gx = 0.5;     % [m] (Same as dx??)
g.gy = 0.2;     % [m] (Same as dy??)
L_i0 = 0.1;     % [m] Initial/neutral actuator position

%data = readtable("LPM3_Spa_Telemetry_Data 3.csv","Delimiter",';');
data = readtable('Telemetry Data/Telemetry_ACC_Spa_991_v2.csv', Delimiter=',');

SimTime = data.Timestamp(end);
accHeave = data.accHeave;
accSway = data.accSway;
accSurge = data.accSurge;
%accRoll = data.accRoll;
%accYaw = data.accYaw;
accRoll = data.oriRoll;
accYaw = data.oriYaw;
oriPitch = data.oriPitch;

t = seconds(data.Timestamp - data.Timestamp(1));
t = seconds(t);
g_a = p.g;
a_Heave = timeseries(accHeave,data.Timestamp);
a_Sway = timeseries(accSway,data.Timestamp);
a_Surge = timeseries(accSurge,data.Timestamp);

a_Roll = timeseries(accRoll,data.Timestamp);      
a_Yaw = timeseries(accYaw,data.Timestamp);         
a_Pitch = timeseries(oriPitch,data.Timestamp);     

% GAINS
k_surge = 0.15;
k_sway = 0.05;
k_heave = 0.05;


max_deg_Surge = 10;
max_deg_Sway = 5;

Ts = timeseries(t,t);
ts_dt = timeseries([0; diff(data.Timestamp)], data.Timestamp);
dt = 1/60;
%Ts = timeseries([0; diff(data.Timestamp)], data.Timestamp);

% Update parameters with actual values
dx = g.dx;
dy = g.dy;
gx = g.gx;
gy = g.gy;
thickness = g.t;

% Actuator
L_max = 0.1;        % [m] Max stroke length 200mm
L_min = -0.1;       % [m]
max_speed = 0.28;       % [m/s] Max actuator speed
max_acc = 2*g_a;          % [m/s^2] Max acceleration

% Actuator time constant
% Time constant is set/chosen from the closed loop bandwidth, typically
% between 10-50 Hz. Check datasheet.
tau_act = 0.03;     % OK servo time constant, try different values
a_denum = exp(-dt/tau_act);
a_num = 1 - a_denum;


%%

35/(2*pi)

1/(0.02*2*pi)

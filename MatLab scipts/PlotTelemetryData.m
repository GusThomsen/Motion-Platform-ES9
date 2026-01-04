
clear;
data = readtable('Telemetry Data/Telemetry_ACC_Spa_991_v2.csv', Delimiter=',');


SimTime = data.Timestamp(end) - data.Timestamp(1);
accHeave = data.accHeave;
accSway = data.accSway;
accSurge = data.accSurge;
accRoll = data.oriRoll;
accYaw = data.oriYaw;
oriPitch = data.oriPitch;

t = seconds(data.Timestamp - data.Timestamp(1));
t = seconds(t);
a_Heave = timeseries(accHeave,t);
a_Sway = timeseries(accSway,t);
a_Surge = timeseries(accSurge,t);

a_Roll = timeseries(accRoll,t);      
a_Yaw = timeseries(accYaw,t);         
a_Pitch = timeseries(oriPitch,t);     
%% 
% Translational data

figure;
subplot(3,1,1)
plot(t, accHeave);
legend('Heave','Location','southeast')
ylabel('[m/s^2]');
xlim([0 data.Timestamp(end)])
grid on;

subplot(3,1,2);
plot(t, accSway,'r');
legend('Sway','Location','southeast')
ylabel('[m/s^2]');
xlim([0 data.Timestamp(end)])
grid on;

subplot(3,1,3);
plot(t, accSurge,'Color', [0 0.5 0]);
legend('Surge','Location','southeast')
ylabel('[m/s^2]');
xlabel('Time [s]')
xlim([0 data.Timestamp(end)])
grid on;

% Rotational data
figure;
subplot(3,1,1);
plot(t, accRoll);
legend('Roll','Location','southeast')
ylabel('degrees');
xlim([0 data.Timestamp(end)])
grid on;

subplot(3,1,2);
plot(t, oriPitch, 'r');
legend('Pitch','Location','southeast')
ylabel('degrees');
xlim([0 data.Timestamp(end)])
grid on;

subplot(3,1,3);
plot(t, accYaw,'Color', [0 0.5 0]);
legend('Yaw','Location','southeast')
ylabel('degrees');
xlabel('Time [s]')
xlim([0 data.Timestamp(end)])
grid on;
 clear ; clc; close all;

%% Controller Parameter Definitions

 kp =35;
 kd =50;

%% Run Simulation

simOut =  sim('Part_3','SimulationMode','normal','AbsTol','1e-6','StopTime', '15',...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');



X_desired = simOut.get('X_desired');
X_actual = simOut.get('X_actual');
Theta_actual = simOut.get('Theta_actual');
ControlTorque = simOut.get('ControlTorque');

%% Make 3d Plots

% fwd kin for joint locations
L1 = 1; L2 = .5;
jointPos = @(t1,t2,t3)[[    0, 0, 0, L1*cos(t1)*cos(t2), cos(t1)*(L2*cos(t2 + t3) + L1*cos(t2))]
[    0, 0, 0, L1*cos(t2)*sin(t1), sin(t1)*(L2*cos(t2 + t3) + L1*cos(t2))]
[ -1/4, 0, 0,         L1*sin(t2),           L2*sin(t2 + t3) + L1*sin(t2)]];

for i=1:50:length(X_desired)
figure(1)
subplot(1,2,1)
jP = jointPos(Theta_actual(i,2),Theta_actual(i,3),Theta_actual(i,4));
plot3(jP(1,:), jP(2,:), jP(3,:),'k','LineWidth',4)
hold on;
plot3(X_desired(1:i,2),X_desired(1:i,3),X_desired(1:i,4),'k')
plot3(X_actual(1:i,2),X_actual(1:i,3),X_actual(1:i,4),'r')
hold off;
axis([-1.5,1.5,-1.5,1.5,-1,2])

subplot(1,2,2)
plot(X_actual(1:i,1),sqrt(sum((X_actual(1:i,:)-X_desired(1:i,:)).^2,2)));
title('Rms Position Error')
xlabel('time')
ylabel('error [cm]')
xlim([0,X_actual(end,1)]);

pause(.05)
end



 
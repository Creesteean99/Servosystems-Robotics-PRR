clc, clear, close all

%% --------------------------DATI-ROBOT------------------------------------
% Si è in singolarità quando b = 90°

L = [0.3, 0.12, 0.45, 0.37];       % L1, L2, L3, L4 [m]
r = [0.01, 0.01, 0.01, 0.01];     % raggi dei link  [m] (utilizzati solo su Simscape) 

m = [0.7 1.4 1 0.9]; 

% Limite sui gunti---------------------------------------------------------
% Supponiamo che:   Alpha spazi tra -60° e +240°
%                   Beta  spazi tra -80° e +80°
%                   Rho   spazi tra  0   e +250 [mm]

amin = -60*pi/180;  amax = 240*pi/180;
bmin = -80*pi/180; bmax = 80*pi/180;
pmin = 0; pmax = 0.250;

Qlim = [amin amax       % amin, amax
        bmin bmax       % bmin, bmax
        pmin pmax];     % pmin, pmax

% Limiti velocità e accelerazioni dei giunti. Angoli in rad, rho in m

ap_max = 4.7;      bp_max = 5.15;      rhop_max = 1; %rad/s m/s

app_max = 4.22;     bpp_max = 6.45;     rhopp_max = 4; %rad/s^2 m/s^2

Qp_const = [ap_max bp_max rhop_max];
Qpp_const = [app_max bpp_max rhopp_max];

%% ------------------------SINGOLARITà-------------------------------------

proSingolarita(proJs())

%% ----------------------PLOT-AREA-E-TRAIETTORIA---------------------------  
% Plot del Robot e dell'area di lavoro

plotAreaPro(Qlim,L);
figure(11)
plotPro([0.125 pi/2 0],L,'k')
figure(12)
plotPro([0.125 pi/2 0],L,'k')

%% ---------------------Creazione della lettera B--------------------------

x1 = 0.60; z1 = 0.90;                     % riferimento X-Z primo punto
x2 = 0.900; z2 = 0.50;                    % riferimento X-Z secondo punto
m_pi = (z1-z2)/(x1-x2);                   % coefficiente angolare
Punti = proLettera(m_pi,x2,z2);

%% -----------------------Traiettoria-Linee-Parabole-----------------------
[S_rp,Sp_rp,Spp_rp,Tt]=proLineeParab(Qp_const, Qpp_const, [1 1 1], Punti); % [1 1 1] acc max di 1m/s^2 per asse cartesiano

% Conversione da spazio di lavoro a spazio dei giunti per successivo studio
% di cinematica

Qt=[];
Qtp=[];
Qtpp=[];

for i = 1:length(Tt)
    Qt(:,i) = proInv(S_rp(:,i),L);
    Jb = proJ(Qt(:,i),L);
    Qtp(:,i) = Jb^-1*Sp_rp(:,i);
    Jbp = proJp(Qt(:,i),Qtp(:,i),L);
    Qtpp(:,i) = Jb^-1*(Spp_rp(:,i)-Jbp*Qtp(:,i));
end

%% ----------------------------KINEMATICS----------------------------------

Home = [0.625 0 0.370]';
Initial_Letter = [Punti(1,1) Punti(2,1) Punti(3,1)]';
Last_Letter = [Punti(1,end) Punti(2,end) Punti(3,end)]';
Qi = proInv(Home,L);

[Qc, Qcp, Qcpp, dtc] = proCicloidale(L,Qp_const,Qpp_const,Home,Initial_Letter);

[Qf, Qfp, Qfpp, dtc2] = proCicloidale(L, Qp_const,Qpp_const,Last_Letter,Home);

[S1, Sp1, Spp1, Tq1,Q_debug1, Qp_debug1, Qpp_debug1, power_real1, Etot1] = proCinematica(L,m,Qc,Qcp,Qcpp,dtc);

[S2, Sp2, Spp2, Tq2,Q_debug2, Qp_debug2, Qpp_debug2, power_real2, Etot2] = proCinematica(L,m,Qt,Qtp,Qtpp,Tt);

[S3, Sp3, Spp3, Tq3,Q_debug3, Qp_debug3, Qpp_debug3, power_real3, Etot3] = proCinematica(L,m,Qf,Qfp,Qfpp,dtc2);

%% -------------------ROBOT-FINALE-----------------------------------------
figure
plotPro(Qc(:,1),L,'b')
hold on, grid on;
plot3(S1(1,:),S1(2,:),S1(3,:))
plot3(S2(1,:),S2(2,:),S2(3,:))
plot3(S3(1,:),S3(2,:),S3(3,:))
l=L(1)+L(2)+L(3)+L(4);
lim = 0.9;
axis equal


Ts1 = mean(diff(dtc));     % passo del primo e terzo segmento
Ts2 = mean(diff(Tt));      % passo del secondo segmento

tt1 = dtc(:);  % parte da 0
tt2 = Tt(:) + tt1(end) + Ts1;  % continua dal punto in cui finisce il primo
tt3 = dtc2(:) + tt2(end) + Ts2;  % continua dal punto in cui finisce il secondo
tt_all = [tt1; tt2; tt3];

pos_all = [Qc(:,:)'; Qt(:,:)'; Qf(:,:)']';
vel_all = [Qcp(:,:)'; Qtp(:,:)'; Qfp(:,:)']';
acc_all = [Qcpp(:,:)'; Qtpp(:,:)'; Qfpp(:,:)';]';
torq_all = [Tq1(:,:)'; Tq2(:,:)'; Tq3(:,:)']';

S_all = [S1(:,:)';S2(:,:)';S3(:,:)'];
V_all = [Sp1(:,:)';Sp2(:,:)';Sp3(:,:)'];
A_all = [Spp1(:,:)';Spp2(:,:)';Spp3(:,:)'];

power_j1 = [power_real1(1,:)';power_real2(1,:)';power_real3(1,:)'];
power_j2 = [power_real1(2,:)';power_real2(2,:)';power_real3(2,:)'];
power_j3 = [power_real1(3,:)';power_real2(3,:)';power_real3(3,:)'];
power_ext = [power_real1(4,:)';power_real2(4,:)';power_real3(4,:)'];

power_tot = power_j1 + power_j2 + power_j3 + power_ext;

%% -------------------Debug cinematica-------------------------------------

figure(104)
subplot(3,1,1)
plot(tt_all,pos_all(1,:),'b')
hold on
plot(tt1, Q_debug1(1,:),'--c')
plot(tt2, Q_debug2(1,:),'--c')
plot(tt3, Q_debug3(1,:),'--c')
grid on
legend('Rho','Rho_{debug}')
xlabel('[s]')
ylabel('Rho[m]')

subplot(3,1,2)
plot(tt_all,pos_all(2,:),'r')
hold on
plot(tt1, Q_debug1(2,:),'--y')
plot(tt2, Q_debug2(2,:),'--y')
plot(tt3, Q_debug3(2,:),'--y')
grid on
legend('Alfa','Alfa_{debug}')
xlabel('[s]')
ylabel('Alfa[rad]')

subplot(3,1,3)
plot(tt_all,pos_all(3,:),'g')
hold on
plot(tt1, Q_debug1(3,:),'--m')
plot(tt2, Q_debug2(3,:),'--m')
plot(tt3, Q_debug3(3,:),'--m')
grid on
legend('Beta','Beta_{debug}')
xlabel('[s]')
ylabel('Betapp[rad]')

figure(105)
subplot(3,1,1)
plot(tt_all,vel_all(1,:),'b')
hold on
plot(tt1(1:end-1), Qp_debug1(1,:),'--c')
plot(tt2(1:end-1), Qp_debug2(1,:),'--c')
plot(tt3(1:end-1), Qp_debug3(1,:),'--c')
grid on
legend('Rhop','Rhop_{debug}')
xlabel('[s]')
ylabel('Rhop[m/s]')

subplot(3,1,2)
plot(tt_all,vel_all(2,:),'r')
hold on
plot(tt1(1:end-1), Qp_debug1(2,:),'--y')
plot(tt2(1:end-1), Qp_debug2(2,:),'--y')
plot(tt3(1:end-1), Qp_debug3(2,:),'--y')
grid on
legend('Alfap','Alfap_{debug}')
xlabel('[s]')
ylabel('Alfap[rad/s]')

subplot(3,1,3)
plot(tt_all,vel_all(3,:),'g')
hold on
plot(tt1(1:end-1), Qp_debug1(3,:),'--m')
plot(tt2(1:end-1), Qp_debug2(3,:),'--m')
plot(tt3(1:end-1), Qp_debug3(3,:),'--m')
grid on
legend('Betap','Betap_{debug}')
xlabel('[s]')
ylabel('Betap[rad/s]')

figure(106)
subplot(3,1,1)
plot(tt_all,acc_all(1,:),'b')
hold on
plot(tt1(1:end-2), Qpp_debug1(1,:),'--c')
plot(tt2(1:end-2), Qpp_debug2(1,:),'--c')
plot(tt3(1:end-2), Qpp_debug3(1,:),'--c')
grid on
legend('Rhopp','Rhopp_{debug}')
xlabel('[s]')
ylabel('Rhop[m/s^2]')

subplot(3,1,2)
plot(tt_all,acc_all(2,:),'r')
hold on
plot(tt1(1:end-2), Qpp_debug1(2,:),'--y')
plot(tt2(1:end-2), Qpp_debug2(2,:),'--y')
plot(tt3(1:end-2), Qpp_debug3(2,:),'--y')
grid on
legend('Alfapp','Alfapp_{debug}')
xlabel('[s]')
ylabel('Alfapp[rad/s^2]')

subplot(3,1,3)
plot(tt_all,acc_all(3,:),'g')
hold on
plot(tt1(1:end-2), Qpp_debug1(3,:),'--m')
plot(tt2(1:end-2), Qpp_debug2(3,:),'--m')
plot(tt3(1:end-2), Qpp_debug3(3,:),'--m')
grid on
legend('Betapp','Betapp_{debug}')
xlabel('[s]')
ylabel('Betapp[rad/s^2]')


%% -------------------SIMSCAPE---------------------------------------------
Qtest = Simulink.SimulationData.Dataset;
ts = timeseries(pos_all,tt_all);
Qtest =addElement(Qtest,ts,'POS');
save('Qtest.mat','Qtest')

Qptest = Simulink.SimulationData.Dataset;
ts = timeseries(vel_all,tt_all);
Qptest = addElement(Qptest, ts, 'VEL');  
save('Qptest.mat', 'Qptest');

Qpptest = Simulink.SimulationData.Dataset;
ts = timeseries(acc_all,tt_all);
Qpptest = addElement(Qpptest, ts, 'ACC');
save('Qpptest.mat', 'Qpptest');

Tqtest = Simulink.SimulationData.Dataset;
ts = timeseries(torq_all, tt_all);
Tqtest = addElement(Tqtest, ts, 'TORQ');
save('Tqtest.mat', 'Tqtest');

Stest = Simulink.SimulationData.Dataset;
ts = timeseries(S_all,tt_all);
Stest =addElement(Stest,ts,'POS_gr');
save('Stest.mat','Stest')

Sptest = Simulink.SimulationData.Dataset;
ts = timeseries(V_all,tt_all);
Sptest =addElement(Sptest,ts,'VEL_gr');
save('Sptest.mat','Sptest')

Spptest = Simulink.SimulationData.Dataset;
ts = timeseries(A_all,tt_all);
Spptest =addElement(Spptest,ts,'ACC_gr');
save('Spptest.mat','Spptest')
%% -------------------------Plot------------------------------------------
figure
plot(tt_all,pos_all),grid on
title('Posizioni giunti traiettoria completa')
legend('Rho','Alfa','Beta')
xlabel('Time [s]')
ylabel('[rad] [m]')
figure
plot(tt_all,vel_all),grid on
title('Velocità giunti traiettoria completa')
legend('Rhop','Alfap','Betap')
xlabel('Time [s]')
ylabel('[rad/s] [m/s]')
figure
plot(tt_all,acc_all),grid on
title('Accelerazioni giunti traiettoria completa')
legend('Rhopp','Alfapp','Betapp')
xlabel('Time [s]')
ylabel('[rad/s^2] [m/s^2]')
figure
plot(tt_all,S_all),grid on
title('Posizione cartesiana traiettoria completa')
legend('X','Y','Z')
xlabel('Time [s]')
ylabel('[m]')
figure
plot(tt_all,V_all),grid on
title('Velocità cartesiana traiettoria completa')
legend('Xp','Yp','Zp')
xlabel('Time [s]')
ylabel('[m/s]')
figure
plot(tt_all,A_all),grid on
title('Accelerazione cartesiana traiettoria completa')
legend('Xpp','Ypp','Zpp')
xlabel('Time [s]')
ylabel('[m/s^2]')
%% coppie-forze
figure
subplot(3,1,1)
plot(tt1,Tq1(1,:),'b',tt2,Tq2(1,:),'b',tt3,Tq3(1,:),'b')
legend('Rho')
xlabel('[s]')
ylabel('[N]')
grid on
subplot(3,1,2)
plot(tt1,Tq1(2,:),'y',tt2,Tq2(2,:),'y',tt3,Tq3(2,:),'y')
legend('Alfa')
xlabel('[s]')
ylabel('[Nm]')
grid on
subplot(3,1,3)
plot(tt1,Tq1(3,:),'r',tt2,Tq2(3,:),'r',tt3,Tq3(3,:),'r')
legend('Beta')
xlabel('[s]')
ylabel('[Nm]')
grid on
%%
figure
plot(tt_all,torq_all),grid on
title('Coppie traiettoria completa')
legend('Rho','Alfa','Beta')
xlabel('Time [s]')
ylabel('[N*m] [N]')
%% potenza 
figure
plot(tt_all,power_j1),grid on
hold on
plot(tt_all,power_j2)
plot(tt_all,power_j3)
plot(tt_all,power_ext)
plot(tt_all,power_tot)
title('Potenze')
legend('J1','J2','J3','Ext','Tot')
xlabel('Time [s]')
ylabel('Power [W]')
%% debug forze
dt = tt_all(2:end)-tt_all(1:end-1);
Etot = [Etot1 Etot2 Etot3];
power_teorica = diff(Etot(:))./dt;

figure
plot(tt_all,power_tot,'k')
hold on
plot(tt_all(1:end-1),power_teorica,'--r')
grid on
legend('Potenza ottenuta','Potenza teorica')
xlabel('[s]')
ylabel('[W]')
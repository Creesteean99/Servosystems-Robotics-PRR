function plotAreaPro(Qlim,L)

amin= Qlim(1,1); amax= Qlim(1,2); 
bmin= Qlim(2,1); bmax= Qlim(2,2); 
pmin= Qlim(3,1); pmax= Qlim(3,2);

n=1001;

a = linspace(amin,amax,n);
b = linspace(bmin,bmax,n);
p = linspace(pmin,pmax,n);  

%--------------------------------------------------------------------------

% scelta degli angoli per calcolo dell'area di lavoro


%--------------------------------------------------------------------------

% center of working space

cwa = (amin+amax)/2; cwb = (bmin+bmax)/2; cwp = (pmin+pmax)/2; 
CW = proDir([cwp cwa cwb], L);

%--------------------------------------------------------------------------

% calcolo minimo e massimo di traiettoria per assi X-Z e X-Y 

for i=1:n

    % X-Z
    S_a_90_p_min(:,i)=proDir([pmin pi/2 b(i)],L);
    S_a_90_p_max(:,i)=proDir([pmax pi/2 b(i)],L);
    S_a_90_b_min(:,i)=proDir([p(i) pi/2 bmin],L);
    S_a_90_b_max(:,i)=proDir([p(i) pi/2 bmax],L);


    % X-Y
    S_a_0_p_min(:,i)=proDir([pmin 0 b(i)],L);
    S_a_0_p_max(:,i)=proDir([pmax 0 b(i)],L);
    S_a_pi_p_min(:,i)=proDir([pmin pi b(i)],L);
    S_a_pi_p_max(:,i)=proDir([pmax pi b(i)],L);

    S_a_0_b_min(:,i)=proDir([p(i) 0 bmin],L);
    S_a_0_b_max(:,i)=proDir([p(i) 0 bmax],L);
    S_a_pi_b_min(:,i)=proDir([p(i) pi bmin],L);
    S_a_pi_b_max(:,i)=proDir([p(i) pi bmax],L);

end

%--------------------------------------------------------------------------

% plot X-Y 

figure(11)
hold on
plot(S_a_0_p_min(1,:),S_a_0_p_min(2,:), 'k', 'LineWidth', 3)
plot(S_a_0_p_max(1,:),S_a_0_p_max(2,:), 'k', 'LineWidth', 3)
plot(S_a_pi_p_min(1,:),S_a_pi_p_min(2,:), 'k', 'LineWidth', 3)
plot(S_a_pi_p_max(1,:),S_a_pi_p_max(2,:), 'k', 'LineWidth', 3)

plot(S_a_0_b_min(1,:),S_a_0_b_min(2,:), 'k', 'LineWidth', 3)
plot(S_a_0_b_max(1,:),S_a_0_b_max(2,:), 'k', 'LineWidth', 3)
plot(S_a_pi_b_min(1,:),S_a_pi_b_min(2,:), 'k', 'LineWidth', 3)
plot(S_a_pi_b_max(1,:),S_a_pi_b_max(2,:), 'k', 'LineWidth', 3)

plot(CW(1), CW(2), '*r', 'Tag','cw')
xlabel('X [m]'), ylabel('Y [m]');
grid on
%axis([0.2 0.8 -0.5 0.5])
axis equal
%daspect([1 1 1])
title("Area Lavoro piano X-Y")

%--------------------------------------------------------------------------

% plot X-Z 

figure(12)

hold on
plot3(S_a_90_p_min(1,:),zeros(length(S_a_90_p_min(1,:))),S_a_90_p_min(3,:), 'k', 'LineWidth', 3)
plot3(S_a_90_p_max(1,:),zeros(length(S_a_90_p_max(1,:))),S_a_90_p_max(3,:), 'k', 'LineWidth', 3)
plot3(S_a_90_b_min(1,:),zeros(length(S_a_90_b_min(1,:))),S_a_90_b_min(3,:), 'k', 'LineWidth', 3)
plot3(S_a_90_b_max(1,:),zeros(length(S_a_90_b_max(1,:))),S_a_90_b_max(3,:), 'k', 'LineWidth', 3)

xlabel('X [m]'), ylabel('Z [m]');

plot3(CW(1),0, CW(3), '*r', 'Tag','cw')
grid on
%axis([0.2 0.8 -0.6 0.6])
axis equal
title("Area Lavoro piano X-Z")

end
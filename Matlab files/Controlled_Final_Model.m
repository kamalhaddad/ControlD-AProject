g=9.81; %gravity 
b = 1;%drag coefficient
L = 0.2; %length of rod
J_pi = 1;%Moment of inertia of the quadcopter
J_L = 0.2;%Moment of Inertia of the load
m_q= 1; %Mass of the Quadcopter
m_L = 0.02 ; %Mass of the load
m_sys = m_q + m_L; %Total mass of the system

%[xdot zdot pidot thetadot z pi theta]'
A = [ 0 0 0 0 0 0 0;
      0 0 0 0 0 0 0;
      0 0 -b/J_pi -b/J_pi 0 0 0;
      0 0 -b/J_L -b/J_L 0 0 0;
      0 1 0 0 0 0 0; 
      0 0 1 0 0 0 0; 
      0 0 0 1 0 0 0];

B = @(State) [
    -sin(State(6))/m_sys 0;
    cos(State(6))/m_sys 0;
    0 1/J_pi;
    -L*sin(State(6) + State(7))/J_L 0;
    0 0; 
    0 0;
    0 0]; 


C = @(State) [
    0;
    -g;
    0;
    -g*sin(State(7))/L;
    0;
    0;
    0];

StateP = @(State,thrust) A*State +B(State)*thrust + C(State);

linThrustFeedback = @(K,K2,State, ref) -K*State + [g*m_sys 0]' + K2*ref;

%% Pole placement

%State = [x' z' pitch' theta' z pitch theta ]'

Zparams = [8 0.6]'; %wn zeta
pole = -Zparams(2)*Zparams(1)+sqrt(1-Zparams(2)^2)*Zparams(1)*1i;
Kz = place([0 0; 1 0], [1/m_sys; 0], [pole conj(pole)]');

Angparams = [8 0.6]'; %wn zeta
pole = -Angparams(2)*Angparams(1)+sqrt(1-Angparams(2)^2)*Angparams(1)*1i;
KA = place([-b/J_pi -b/J_pi 0 0; -b/J_L -b/J_L 0 -g/L; 1 0 0 0; 0 1 0 0], ...
    [1/J_pi; 0; 0; 0], [pole conj(pole) -10 -14]');

%% LQR
Kz = lqr([0 0; 1 0], [1/m_sys; 0], [200,0;0,1000],1);

KA = lqr([-b/J_pi -b/J_pi 0 0; -b/J_L -b/J_L 0 -g/L; 1 0 0 0; 0 1 0 0], ...
    [1/J_pi; 0; 0; 0], [1 0 0 0;0 250 0 0;0 0 300 0;0 0 0 100],1);

%% Simulation

K = [0 Kz(1) 0 0 Kz(2) 0 0; 0 0 KA(1) KA(2) 0 KA(3) KA(4)]; %gains matrix

K2 = [0; 0];
ref = 0;
dt = 1e-3;
tend = 10;

State = [0 0 0 0 -0.2 0 0]';
States = zeros(7, tend/dt + 1);
States(:, 1) = State;
StatePs = zeros(7, tend/dt);

Us = zeros(2, tend/dt);

disturbance = @() 0;
%Uncomment to add noise
%disturbance = @() random('norm', 0, 0.01);

for n = 1:tend/dt
    U = -K*State + [g*m_sys 0]';
    P = StateP(State, U);
    D = [0 1 0 0 0 0 0]' * disturbance();
    State = State + ...
        dt * P + D;
    States(:, 1 + n) = State ;
    StatePs(:, n) = P;
    Us(:,n) = U;
end
% State = [x' z' pitch' theta' z pitch theta ]'

%Plotting graphs
figure(1)
plot(0:dt:tend, States(1,:),'r');
hold on
plot(0:dt:tend, States(2,:),'b');
hold off
title('xdot and zdot')

figure(2)
plot(0:dt:tend, States(3,:),'r');
hold on
plot(0:dt:tend, States(4,:),'b');
hold off
title('pidot and thetadot')

figure(3)
plot(0:dt:tend, States(5,:));
title('z')

figure(4)
plot(0:dt:tend, States(6,:),'r');
hold on
plot(0:dt:tend, States(7,:),'b');
hold off
title('Pi and theta');

figure(5)
plot(0:dt:tend-dt, StatePs(1,:),'r');
hold on
plot(0:dt:tend-dt, StatePs(2,:),'b');
hold off
title('xdd and zdd');

figure(6)
plot(0:dt:tend-dt, StatePs(3,:),'r');
hold on
plot(0:dt:tend-dt, StatePs(4,:),'b');
hold off
title('pidd and thetadd');

figure(7)
plot(0:dt:tend-dt, Us(1,:),'r');
hold on
plot(0:dt:tend-dt, Us(2,:),'b');
hold off
title('F1 and F3');

%% Phase plane

apt = A(3:4,3:4);
figure(4);
phasePlane(apt, [0 0]');
title('theta dot vs pi dot');
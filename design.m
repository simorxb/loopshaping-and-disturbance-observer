%% Controller Design

% Define system parameters
m = 10;
k = 0.5;

% Transfer function variable
s = tf('s');

% System transfer function
G = 1/(s*(m*s + k));

%% Controller synthesis - loop shaping - loopsyn command

% alpha specifies the tradeoff between performance and robustness in the interval [0,1].
% Smaller alpha favors performance (mixsyn design) and larger alpha favors robustness (ncfsyn design).
alpha = 0.5;

% Desired transfer function (setpoint to output)
tau = 0.2;
Td = 1/(tau*s+1)^2;

% Calculate Ld using Td
% Ld = Td/(1-Td)
Ld = 1/(tau^2*s^2 + 2*tau*s);

% Compute K using loopsyn loop shaping
[K, CL, gamma, info] = loopsyn(G, Ld, alpha);

% Find K's transfer function
K_tf = tf(K)

% Open loop function's Bode diagram to check stability margins
figure;
margin(K_tf*G); % Plot Bode diagram with gain and phase margins
set(findall(gcf,'type','line'),'LineWidth',2);
grid on;

% Closed-loop transfer function
G_L = feedback(K_tf * G, 1);

% Print G_L poles
disp('Poles of the closed-loop transfer function:');
poles = pole(G_L);
disp(poles);

% Plot root locus
figure;
rlocus(K_tf * G);
title('Root Locus');

% Add poles to root-locus
hold on;
plot(real(poles), imag(poles), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
hold off;

% Calculate the gain and phase margins
[gm, pm, wcg, wcp] = margin(K_tf * G);
gm_dB = 20 * log10(gm);

% Print margins
fprintf('Gain Margin: %.3g dB at frequency %.3g rad/sec\n', gm_dB, wcg);
fprintf('Phase Margin: %.3g deg at frequency %.3g rad/sec\n', pm, wcp);
fprintf('Delay Margin: %.3g seconds\n', (pm * pi / 180) / wcp);

% Step response plot
figure;
subplot(2, 1, 1);
[y_G_L, T_G_L] = step(G_L);
plot(T_G_L, y_G_L, 'LineWidth', 2);
title('Step Response');
grid on;
xlabel('Time [s]');
ylabel('Position [m]');

% Transfer function from setpoint to control input (torque)
G_L_u = K_tf / (1 + K_tf * G);

% Step response - control input
subplot(2, 1, 2);
[y_G_L_u, T_G_L_u] = step(G_L_u);
plot(T_G_L_u, y_G_L_u, 'LineWidth', 2);
title('Control Input');
grid on;
xlabel('Time [s]');
ylabel('Force [N]');

%% Observer Design

% Extended system dynamics (3rd state is the disturbance)

Ae = [-k/m 0 1/m; 1 0 0; 0 0 0];
Be = [1/m; 0; 0];
Ce = [0 1 0];
De = 0;

% Place observer poles farther left from the slowest closed loop function poles
L = place(Ae', Ce', [-3 -3.5 -4])';

%% Discretize Controller

% Sample time
Ts = 0.1;

% Discretize controller
K_tf_d = c2d(K_tf, Ts, 'matched');

% Discretise Observer
obs = ss(Ae, Be, Ce, De);
obs_d = c2d(obs, Ts);
close all

%% Linear

% Condição de linearização
xlin = [0; 0.905902; -0.0698132; 0; 0.349066; 0];
ulin = [3.48695; 3.48695];

% Definir a matriz A
A = [0, 1, 0, 0, 0, 0;
     0, -0.14549461917889836, -1.8848838586236563, 0, 0, ...
     0.4275284230672521;
     0, 0, 0, 1, 0, 0;
     0, 0, 0, -1.8800697844302587, 0, 0;
     0, 0, 0, 0, 0, 1;
     0, -0.40407110644562066, 0.1241847263016029, 0, ...
     -1.2563699838950038, -0.09167448459795735];

% Definir a matriz B
B = [0, 0;
     0.03584493247672999, 0.03584493247672999;
     0, 0;
     3.4465182869101043, -3.4465182869101043;
     0, 0;
     0.4829741629872683, 0.4829741629872683];

% Definir a matriz C
C = [1, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0;
     0, 0, 0, 0, 1, 0];

% Definir a matriz D
D = [0, 0;
     0, 0;
     0, 0];

% Definir o vetor constante d0
d0 = [0.9059023616211557; 0; 0; 0; 0; 0];

% Configurar o tempo da simulação
tspan = [0:0.01:100];  % Tempo inicial e final
x0 = xlin - xlin;  % Condições iniciais
% x0 = [0; 2; -0.2; 0; 0.5; 0] - xlin;  % Condições iniciais

% Resolver o sistema de equações diferenciais usando ode45
[t, x] = ode45(@(t, x) A*x + B*u(t) + d0, tspan, x0);

% Calcular as saídas do sistema
y = C * x';  % Vetor x precisa ser transposto para o plot

%% Não Linear

% Condição inicial
x0nl = xlin;  % Condições iniciais
% x0 = [0; 2; -0.2; 0; 0.5; 0];  % Condições iniciais

% Configurar o tempo da simulação
tspan = [0:0.01:100];  % Tempo inicial e final

% Resolver o sistema de equações diferenciais usando ode45
[t, x] = ode45(@(t, x) nonlinear_dynamics(t, x, unl(t,ulin)), tspan, x0nl);

% Calcular as saídas do sistema
ynl = [x(:, 1), x(:, 3), x(:, 5)]';

%% Plots
% Criar gráficos para os valores das saídas do sistema
figure(1);
subplot(3,1,1);
plot(t, 180/pi * ynl(1, :), 'g', 'LineWidth', 2);
hold on
plot(t, 180/pi*(y(1, :) + xlin(1,1)), 'k', 'LineWidth', 1);
xlabel('Tempo (s)');
ylabel('Angulo (°)');
legend('Modelo linear', 'Modelo não linear', 'Location', 'southeast');
title('Ângulo Azimutal');
grid on;

subplot(3,1,2);
plot(t, 180/pi * ynl(2, :), 'b', 'LineWidth', 2);
hold on
plot(t, 180/pi*(y(2, :) + xlin(3,1)), 'k', 'LineWidth', 1);
xlabel('Tempo (s)');
ylabel('Angulo (°)');
legend('Modelo linear', 'Modelo não linear', 'Location', 'southeast');
title('Ângulo de Arfagem');
grid on;

subplot(3,1,3);
plot(t, 180/pi * ynl(3, :), 'r', 'LineWidth', 2);
hold on
plot(t, 180/pi*(y(3, :) + xlin(5,1)), 'k', 'LineWidth', 1);
xlabel('Tempo (s)');
ylabel('Angulo (°)');
legend('Modelo linear', 'Modelo não linear', 'Location', 'southeast');
title('Ângulo de Elevação');
grid on;

sgtitle('Comparação dos ângulos entre modelos linear e não-linear');

% Calcular e plotar a entrada de controle u(t)
u_values = zeros(length(t), 2);

for i = 1:length(t)
    u_values(i, :) = unl(t(i), ulin);  % Avaliar u(t) em cada instante de tempo t
end

% Criar nova janela de figura para o plot de u(t)
figure(2);
subplot(2,1,1);
plot(t, u_values(:,1) * 1000, 'b', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Tensão (mV)');
legend('Entrada u1', 'Location', 'southwest');
grid on;

subplot(2,1,2);
plot(t, u_values(:,2) * 1000, 'r', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Tensão (mV)');
legend('Entrada u2', 'Location', 'southwest');
grid on;

sgtitle('Entrada degrau');

% Definir a função de controle variável
function control_input = unl(t,ulin)
    % Exemplo de controle variável no tempo
    if t < 20
        control_input = [0;0] + ulin;  % Valores constantes
    elseif t > 50
        control_input = [0;0] + ulin;
    else
        control_input = 10^-3 * [99.5; 99] + ulin;
    end
end

% Função para entrada de controle variável e definida por partes
function control_input = u(t)
    % Exemplo de controle variável no tempo
    if t < 20
        control_input = 10^-3*[0; 0];  % Valores constantes
        % Valores dependentes do tempo
    elseif t > 50
        control_input = 10^-3*[0; 0];
    else
        control_input = 10^-3*[50; 45];
    end
end

% Dinâmica não linear do sistema
function dxdt = nonlinear_dynamics(~, x, u)
    % Extraindo variáveis de estado e entradas de controle
    x2 = x(2); x3 = x(3); x4 = x(4); x5 = x(5); x6 = x(6);
    u1 = u(1); u2 = u(2);
    
    % Definindo as equações do sistema
    dx1 = x2;
    dx2 = (-0.147726 * u1^2 * sin(x3) - 0.0594077 * u1 * sin(x3) ...
           - 0.147726 * u2^2 * sin(x3) - 0.0594077 * u2 * sin(x3) ...
           + x2 * (x6 * (2.17335 * sin(2 * x5) - 0.517289 * cos(2 * x5)) - 0.30852)) ...
           / (-1.08667 * (sin(x5))^2 + 0.258644 * sin(2 * x5) + 1.08667 * (cos(x5))^2 + 1.12179);
    dx3 = x4;
    dx4 = 0.467258 * u1^2 + 0.187907 * u1 - 0.467258 * u2^2 - 0.187907 * u2 - 1.88007 * x4;
    dx5 = x6;
    dx6 = 0.0656387 * u1^2 * cos(x3) + 0.0263965 * u1 * cos(x3) + 0.0656387 * u2^2 * cos(x3) ...
           + 0.0263965 * u2 * cos(x3) - 0.115282 * x2^2 * (sin(x5))^2 - 0.484347 * x2^2 * sin(2 * x5) ...
           + 0.115282 * x2^2 * (cos(x5))^2 - 1.03886 * sin(x5) - 1.31701 * cos(x5) - 0.0916745 * x6;
    
    % Vetor de derivadas
    dxdt = [dx1; dx2; dx3; dx4; dx5; dx6];
end
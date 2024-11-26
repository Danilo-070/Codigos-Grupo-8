close all;
clear;
clc;

%% Espaco de estados
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

% Definido o EE
sys = ss(A,B,C,D);

%% Polos e zeros
polos = eig(A);
sys_tf = tf(sys);

% Mapa de polos e zeros 
figure(1);clf
pzplot(sys_tf);
grid on;
title('Mapa de Polos e Zeros');
xlabel('Parte Real');
ylabel('Parte Imaginária');

% Frequencias naturais e amortecimento
[wn,zeta] = damp(sys);

%% Criterio de Estabilidade de Routh-Hurwitz
pol_A = poly(A);
rhc(pol_A); % Avaliacao da estabilidade com o polinomio completo
rhc(pol_A(1:5)); % Avaliacao da estabilidade com s² em evidencia

%% Funcoes de transferencia
[num1, den1] = ss2tf(A,B,C,D,1);
[num2, den2] = ss2tf(A,B,C,D,2);

% FT's para o rotor 1 (u1)
ft_T_u1 = tf(num1(1,:), den1);
ft_P_u1 = tf(num1(2,:), den1);
ft_E_u1 = tf(num1(3,:), den1);

% FT's para o rotor 2 (u2)
ft_T_u2 = tf(num2(1,:), den2);
ft_P_u2 = tf(num2(2,:), den2);
ft_E_u2 = tf(num2(3,:), den2);

%% Diagramas de Bode
% Relacionados a T/u1
figure(2);clf
bode(ft_T_u1)
margin(ft_T_u1)
grid on

% Relacionados a p/u1
figure(3);clf
bode(ft_P_u1)
margin(ft_P_u1)
grid on

% Relacionados a E/u1
figure(4);clf
bode(ft_E_u1)
margin(ft_E_u1)
grid on

% Relacionados a T/u2
figure(5);clf
bode(ft_T_u2)
margin(ft_T_u2)
grid on

% Relacionados a P/u2
figure(6);clf
bode(ft_P_u2)
margin(ft_P_u2)
grid on

% Relacionados a E/u2
figure(7);clf
bode(ft_E_u2)
margin(ft_T_u2)
grid on

%% Parametros para simulacao
% Definir o vetor constante d0
d0 = [0.9059023616211557; 0; 0; 0; 0; 0];

% Configurar o tempo da simulação
tspan = [0 100];  % Tempo inicial e final
x0 = pi/180*[0; 30; -10; 15; 10; 5] - xlin;  % Condições iniciais em °


%% Simulacao com entrada degrau
% Resolver o sistema de equações diferenciais usando ode45
[t, x] = ode45(@(t, x) A*x + B*u(t) + d0, tspan, x0);

% Calcular as saídas do sistema
y = C * x';  % Vetor x precisa ser transposto para o plot

% Criar gráficos para os valores absolutos (desvio + linearização)
figure(8);clf;
subplot(3,1,1);
plot(t, 180/pi*(y(1, :) + xlin(1,1)), 'g', 'LineWidth', 2);
title('Ângulo Azimutal - T')
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

subplot(3,1,2);
plot(t, 180/pi*(y(2, :) + xlin(3,1)), 'b', 'LineWidth', 2);
title('Ângulo Pitch - P')
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

subplot(3,1,3);
plot(t, 180/pi*(y(3, :) + xlin(5,1)), 'r', 'LineWidth', 2);
title('Ângulo Elevação - E')
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

sgtitle('Resposta a entrada Degrau');

% Calcular e plotar a entrada de controle u(t)
u_values = zeros(length(t), 2);

for i = 1:length(t)
    u_values(i, :) = u(t(i));
end

% Criar nova janela de figura para o plot de u(t)
figure(9);clf;
subplot(2,1,1);
plot(t, (u_values(:,1) + ulin(1,1)), 'b', 'LineWidth', 2);
title('Entrada u1')
xlabel('Tempo (s)');
ylabel('Tensão (V)');
grid on;

subplot(2,1,2);
plot(t, (u_values(:,2) + ulin(2,1)), 'r', 'LineWidth', 2);
title('Entrada u2')
xlabel('Tempo (s)');
ylabel('Tensão (V)');
grid on;

sgtitle('Entradas - Degrau');

%% Simulação com entrada rampa
[t, x] = ode45(@(t, x) A*x + B*u_ramp(t) + d0, tspan, x0);
y = C * x';  % Calcular as saídas do sistema

% Plotar a resposta à entrada rampa
figure(10);clf;
subplot(3,1,1);
plot(t, 180/pi*(y(1, :) + xlin(1,1)), 'g', 'LineWidth', 2);
title('Ângulo Azimutal - T (Rampa)');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

subplot(3,1,2);
plot(t, 180/pi*(y(2, :) + xlin(3,1)), 'b', 'LineWidth', 2);
title('Ângulo Pitch - P (Rampa)');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

subplot(3,1,3);
plot(t, 180/pi*(y(3, :) + xlin(5,1)), 'r', 'LineWidth', 2);
title('Ângulo Elevação - E (Rampa)');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

sgtitle('Resposta a entrada Rampa');

% Plotar o sinal de entrada rampa
u_values = zeros(length(t), 2);
for i = 1:length(t)
    u_values(i, :) = u_ramp(t(i));
end

figure(11);clf;
subplot(2,1,1);
plot(t, (u_values(:,1) + ulin(1,1)), 'b', 'LineWidth', 2);
title('Entrada u1 (Rampa)');
xlabel('Tempo (s)');
ylabel('Tensão (V)');
grid on;

subplot(2,1,2);
plot(t, (u_values(:,2) + ulin(2,1)), 'r', 'LineWidth', 2);
title('Entrada u2 (Rampa)');
xlabel('Tempo (s)');
ylabel('Tensão (V)');
grid on;

sgtitle('Entradas - Rampa');

%% Simulação com entrada impulso
[t, x] = ode45(@(t, x) A*x + B*u_impulse(t) + d0, tspan, x0);
y = C * x';

% Plotar a resposta à entrada impulso
figure(12);clf;
subplot(3,1,1);
plot(t, 180/pi*(y(1, :) + xlin(1,1)), 'g', 'LineWidth', 2);
title('Ângulo Azimutal - T (Impulso)');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

subplot(3,1,2);
plot(t, 180/pi*(y(2, :) + xlin(3,1)), 'b', 'LineWidth', 2);
title('Ângulo Pitch - P (Impulso)');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

subplot(3,1,3);
plot(t, 180/pi*(y(3, :) + xlin(5,1)), 'r', 'LineWidth', 2);
title('Ângulo Elevação - E (Impulso)');
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
grid on;

sgtitle('Resposta a entrada Impulso');

% Plotar o sinal de entrada impulso
u_values = zeros(length(t), 2);
for i = 1:length(t)
    u_values(i, :) = u_impulse(t(i));
end

figure(13);clf;
subplot(2,1,1);
plot(t, (u_values(:,1) + ulin(1,1)), 'b', 'LineWidth', 2);
title('Entrada u1 (Impulso)');
xlabel('Tempo (s)');
ylabel('Tensão (V)');
grid on;

subplot(2,1,2);
plot(t, (u_values(:,2) + ulin(2,1)) , 'r', 'LineWidth', 2);
title('Entrada u2 (Impulso)');
xlabel('Tempo (s)');
ylabel('Tensão (V)');
grid on;

sgtitle('Entradas - Impulso');

%% Simulacao com entrada composta
% Essa entrada é composta com uma rampa no começo, seguida de uma
% perturbação com senoide e por fim uma entrada degrau. A ideia é simular
% a partida gradual de um sistema até um certo nível, após isso o
% helicóptero está sujeito a perturbações e por fim o sistema tem um
% período de tensão constante e depois é desligado.

[t_comp, x_comp] = ode45(@(t, x) A*x + B*u_composite(t), tspan, x0);
y_comp = C * x_comp';

% Plotar a resposta à entrada composta
figure(14);clf;
subplot(3,1,1);
plot(t_comp, 180/pi*(y_comp(1, :) + xlin(1,1)), 'g', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Ângulo Azimutal (°)');
title('Resposta a Entrada Composta - Azimutal');
grid on;

subplot(3,1,2);
plot(t_comp, 180/pi*(y_comp(2, :) + xlin(3,1)), 'b', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
title('Resposta a Entrada Composta - Pitch');
grid on;

subplot(3,1,3);
plot(t_comp, 180/pi*(y_comp(3, :) + xlin(5,1)), 'r', 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
title('Resposta a Entrada Composta - Elevação');
grid on;

sgtitle('Resposta a Entrada Composta');

% Plotar o sinal de entrada composta
u_values = zeros(length(t_comp), 2);
for i = 1:length(t_comp)
    u_values(i, :) = u_composite(t_comp(i));
end

figure(15);clf;
subplot(2,1,1);
plot(t_comp, (u_values(:,1) + ulin(1,1)), 'b', 'LineWidth', 2);
title('Entrada u1 (Composta)');
xlabel('Tempo (s)');
ylabel('Tensão (V)');
grid on;

subplot(2,1,2);
plot(t_comp, (u_values(:,2) + ulin(2,1)) , 'r', 'LineWidth', 2);
title('Entrada u2 (Composta)');
xlabel('Tempo (s)');
ylabel('Tensão (V)');
grid on;

sgtitle('Entradas - Composta');

%% Simulação da resposta do modelo não linear com entrada composta
[t_comp, x_comp_nl] = ode45(@(t, x) nonlinear_dynamics(t, x, u_composite(t)), tspan, x0);
y_comp_nl = [x_comp_nl(:, 1), x_comp_nl(:, 3), x_comp_nl(:, 5)]'; % Saídas do sistema não linear

% Comparação entre o modelo linear e não linear
% Simulação do modelo linear com entrada composta
[t_comp_lin, x_comp_lin] = ode45(@(t, x) A*x + B*u_composite(t), tspan, x0);
y_comp_lin = C * x_comp_lin';  % Saídas do sistema linear

% Plot dos três ângulos comparando o modelo linear e não linear
figure(16); clf;

% Ângulo azimutal - T
subplot(3,1,1);
plot(t_comp, 180/pi * y_comp_nl(1, :), 'k', 'LineWidth', 1); % Modelo não linear
hold on;
plot(t_comp_lin, 180/pi * (y_comp_lin(1, :) + xlin(1,1)), 'g', 'LineWidth', 2); % Modelo linear
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
legend('Modelo Não Linear', 'Modelo Linear', 'Location', 'southeast');
title('Comparação do Ângulo Azimutal (T)');
grid on;

% Ângulo Pitch - P
subplot(3,1,2);
plot(t_comp, 180/pi * y_comp_nl(2, :), 'k', 'LineWidth', 1); % Modelo não linear
hold on;
plot(t_comp_lin, 180/pi * (y_comp_lin(2, :) + xlin(3,1)), 'b', 'LineWidth', 2); % Modelo linear
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
legend('Modelo Não Linear', 'Modelo Linear', 'Location', 'southeast');
title('Comparação do Ângulo Pitch (P)');
grid on;

% Ângulo Elevação - E
subplot(3,1,3);
plot(t_comp, 180/pi * y_comp_nl(3, :), 'k', 'LineWidth', 1); % Modelo não linear
hold on;
plot(t_comp_lin, 180/pi * (y_comp_lin(3, :) + xlin(5,1)), 'r', 'LineWidth', 2); % Modelo linear
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
legend('Modelo Não Linear', 'Modelo Linear', 'Location', 'southeast');
title('Comparação do Ângulo Elevação (E)');
grid on;

sgtitle('Comparação entre os Modelos Linear e Não Linear - Entrada Composta');

%% Funcoes

% Funcao para entrada degrau
function control_input = u(t)
    % Exemplo de controle variável no tempo
    if t < 20
        control_input = [0; 0];
    elseif t > 50
        control_input = [0; 0];
    else
        control_input = [1; 1];
    end
end

% Funcao para entrada rampa
function control_input = u_ramp(t)
    % Exemplo de entrada rampa
    if t < 0
        control_input = [0; 0];
    elseif t > 30
        control_input = [0; 0];
    else
        control_input = [0.1 * (t - 0); 0.1 * (t - 0)]; % Inclinação da rampa
    end
end

% Funcao para entrada impulso
function control_input = u_impulse(t)
    % Exemplo de entrada impulso
    if abs(t - 0) < 0.1  % Impulso centrado em t = 0 s com uma duração curta
        control_input = [10; 10]; % Amplitude do impulso
    else
        control_input = [0; 0];
    end
end

% Funcao para entrada composta
function control_input = u_composite(t)
    if t < 20
        % Rampa do instante 0 até 20 segundos
        control_input = [0.025 * t; 0.025 * t];
    elseif 20 <= t && t <= 60
        % Oscilação senoidal de 20 a 60 segundos
        control_input = [0.5 * sin(2 * pi * 0.1 * (t - 20)); 0.5 * sin(2 * pi * 0.1 * (t - 20))];
    elseif 60 < t && t <= 80
        % Função degrau de 60 a 80 segundos
        control_input = [0.5; 0.5];
    else
        % Sistema sem entrada de 80 a 100 segundos
        control_input = [0; 0];
    end
end

% Funcao para dinamica nao linear do sistema
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

% Funcao para Routh-Hurwitz
function [] = rhc(den)
    % Função para calcular a tabela de Routh-Hurwitz e determinar a estabilidade
    % den: Vetor dos coeficientes do polinômio característico

    % Determina o grau do polinômio e ajusta o número de colunas na tabela
    den_hp = length(den) - 1;
    den_hp_added = den_hp + 1;

    if den_hp == 0
        cols = 1;
    elseif den_hp == 1 || den_hp == 2
        cols = 2;
    elseif den_hp == 3 || den_hp == 4
        cols = 3;
    elseif den_hp == 5 || den_hp == 6
        cols = 4;
    elseif den_hp == 7 || den_hp == 8
        cols = 5;    
    elseif den_hp == 9 || den_hp == 10
        cols = 6;
    elseif den_hp == 11 || den_hp == 12
        cols = 7;
    elseif den_hp == 13 || den_hp == 14
        cols = 8;  
    end

    % Inicializa a tabela de Routh-Hurwitz
    RH_Table = zeros(den_hp + 1, cols);

    % Configura os elementos da primeira e segunda linhas da tabela
    RH_Table(1, 1) = den(1);
    if cols > 1
        RH_Table(2, 1) = den(2);
    end

    first_count = 3;
    second_count = 4;

    % Preenche a primeira linha da tabela
    for i = 2:cols
        if first_count <= den_hp_added
            RH_Table(1, i) = den(first_count);
            first_count = first_count + 2; 
        else 
            RH_Table(1, i) = 0;
        end
    end

    % Preenche a segunda linha da tabela
    for i = 2:cols
        if second_count <= den_hp_added
            RH_Table(2, i) = den(second_count);
            second_count = second_count + 2; 
        else 
            RH_Table(2, i) = 0;
        end
    end

    % Preenche as linhas restantes da tabela de Routh-Hurwitz
    X = zeros(2, 2);
    for i = 1:(den_hp - 1)
        for j = 1:(cols - 1)
            X(1, 1) = RH_Table(i, 1);
            X(2, 1) = RH_Table(i + 1, 1);
            X(1, 2) = RH_Table(i, j + 1);
            X(2, 2) = RH_Table(i + 1, j + 1);
            
            RH_Table(i + 2, j) = -det(X) / RH_Table(i + 1, 1);     
        end 
    end

    % Exibe a tabela de Routh-Hurwitz
    fprintf('\nTabela de Routh-Hurwitz:\n');
    disp(RH_Table);

    % Contar o número de mudanças de sinal na primeira coluna
    Right_poles = 0;
    for i = 1:den_hp
        if sign(RH_Table(i, 1)) * sign(RH_Table(i + 1, 1)) < 0
            Right_poles = Right_poles + 1;
        end
    end

    % Verificar as raízes do polinômio para identificar polos no eixo imaginário
    Raizes = roots(den);
    Imaginary_poles = sum(real(Raizes) == 0);

    % Exibir o número de polos no semiplano direito
    fprintf('\nNúmero de polos no semiplano direito = %2.0f\n', Right_poles);

    % Exibir o número de polos no eixo imaginário
    fprintf('Número de polos no eixo imaginário = %2.0f\n', Imaginary_poles);

    % Exibir a condição de estabilidade
    if Right_poles > 0
        fprintf('O sistema é instável.\n');
    elseif Imaginary_poles > 0
        fprintf('O sistema é marginalmente estável.\n');
    else
        fprintf('O sistema é estável.\n');
    end

    % Exibir as raízes do polinômio
    fprintf('\nPolos do sistema:\n');
    disp(Raizes);
end
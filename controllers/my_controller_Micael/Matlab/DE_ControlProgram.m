%  Controle geral do programa, com etapas bem definidas:
%  Control_program()
%    |    |--> Automatic_reading() <--   <--   <--  <--  <--  <--
%    |    |--> Control_program(metaheuristic) <--> fitness()    /|\
%    |    |--> output                                            |
%    |    |--> Automatic_save()                                  |
%    |    |--> Wait  -->  -->  --> -->  -->  -->  -->   -->  --> |
%   \|/


function DE_ControlProgram(destination_path, population_size, max_generations, F, CR)

    % ---------------------------------------------------------------------
    % Seed creation to ensure random program
    % ---------------------------------------------------------------------
    seed = round(sum(clock * 10000));
    rng(seed);

    population_size = str2double(population_size);
    max_generations = str2double(max_generations);
    F = str2double(F);      % Fator diferencial (escala) -> típico: 0.4 a 0.9
    CR = str2double(CR);    % Probabilidade de crossover -> típico: 0.6 a 0.95

    % ---------------------------------------------------------------------
    % Parâmetros do robô / C-space
    % ---------------------------------------------------------------------
    robotRadius = 0.15;
    alpha = 1.4;

    tempos = [];
    tempos2 = [];
    cont_m_displacement = 0;
    pR = [];

    global Rd
    Rd = 1;  % raio de ação local para busca do próximo ponto (sub-espaço)

    client = tcpclient("localhost", 12345); % TCP connection

    while (1)

        % -------------------------------------------------------------
        % 1) Leitura automática do estado atual do ambiente
        % -------------------------------------------------------------
        [K_points, m_displacement, pO_size, pR, final_objective, pDP, pO] = ...
            automatic_reading(cont_m_displacement, pR, destination_path);

        % C-space (obstáculos inflados + margem de segurança)
        pDP_space = C_space(pDP, robotRadius, alpha);

        % Se já chegou (apenas quando displacement = 0 no início)
        if (m_displacement == 0)
            d = norm(pR(end,:) - final_objective);
            if (d < 0.2)
                break;
            end
        end

        % -------------------------------------------------------------
        % 2) Define a função fitness do MHRTSN
        % -------------------------------------------------------------
        func_fitness = @(x) gJ(x, pDP_space, pO, pO_size, pR, final_objective, 1);

        % Janela de busca local (em torno do robô)
        a = pR(end,1);
        b0 = pR(end,2);

        lb = [(a - Rd), (b0 - Rd)];
        ub = [(a + Rd), (b0 + Rd)];

        % -------------------------------------------------------------
        % 3) Executa DE para encontrar o próximo objetivo local z = (x,y)
        % -------------------------------------------------------------
        tic;
        t1 = cputime;

        [z, fval, de_info] = run_DE(func_fitness, lb, ub, population_size, max_generations, F, CR);

        t2 = cputime - t1;

        tempos2 = [toc, tempos2];
        tempos  = [t2, tempos];

        % -------------------------------------------------------------
        % 4) Salva o ponto escolhido para o deslocamento atual
        % -------------------------------------------------------------
        [cont_m_displacement] = automatic_save(z, cont_m_displacement, destination_path);

        % -------------------------------------------------------------
        % 5) Handshake TCP para garantir leitura do próximo passo
        % -------------------------------------------------------------
        while true
            write(client, "read");
            confirm = read(client, 8, 'char');
            if (confirm == "received")
                break;
            end
            pause(0.5);
        end

        % -------------------------------------------------------------
        % 6) Plot incremental (mantém a mesma janela)
        % -------------------------------------------------------------
        if ~exist('fig', 'var')
            fig = figure;
            hold on;
        end

        for c = 1:pO_size
            plot(pO(c, 1), pO(c, 2), "-o");
        end

        plot(final_objective(1), final_objective(2), 'diamond');
        plot(z(1), z(2), "*");                                 % próximo ponto escolhido
        plot(pR(m_displacement + 1, 1), pR(m_displacement + 1, 2), "square");
        plot(pDP_space(:, 1), pDP_space(:, 2), '-.', 'Color', 'blue');

        % Critério de parada
        d = norm(z - final_objective);
        if (d < 0.2 || cont_m_displacement > 50)
            break;
        end
    end

    % Atualiza rota com o último ponto escolhido
    pR = [pR; z];

    % ---------------------------------------------------------------------
    % 7) Geração de log e salvamento das figuras (igual ao padrão existente)
    % ---------------------------------------------------------------------
    if (cont_m_displacement < 50)
        route_distance = 0;
        max_distance = 0;

        for i = 1:m_displacement + 1
            aux = norm(pR(i + 1,:) - pR(i,:));
            route_distance = route_distance + aux;
            max_distance = max(max_distance, aux);
        end

        sum_tempos = sum(tempos);
        sum_tempos2 = sum(tempos2);

        file = fullfile(destination_path, 'log.txt');
        fileID = fopen(file, 'w');

        fprintf(fileID, 'Dados das Variáveis:\n\n');
        fprintf(fileID, 'Tempo de CPU: \n');
        fprintf(fileID, ' %12.8f \n\n', tempos);
        fprintf(fileID, 'Somatório do tempo de CPU gasto: \n');
        fprintf(fileID, ' %6.8f \n\n', sum_tempos);
        fprintf(fileID, 'Número de deslocamentos: \n');
        fprintf(fileID, ' %d \n\n', m_displacement + 1);
        fprintf(fileID, 'Pontos do robô (centros): \n');
        for i = 1:size(pR, 1)
            fprintf(fileID, '%6.8f %6.8f\n', pR(i, 1), pR(i, 2));
        end
        fprintf(fileID, '\n\n Distância percorrida: \n');
        fprintf(fileID, ' %6.8f \n\n', route_distance);
        fprintf(fileID, 'Somatório do tempo de relógio gasto: \n');
        fprintf(fileID, ' %6.8f \n\n', sum_tempos2);
        fprintf(fileID, 'Semente utilizada neste experimento: \n');
        fprintf(fileID, ' %d \n\n', seed);

        % Log DE (opcional)
        fprintf(fileID, '\n\n--- Informações DE ---\n');
        fprintf(fileID, 'F: %6.4f\n', F);
        fprintf(fileID, 'CR: %6.4f\n', CR);
        fprintf(fileID, 'Melhor fitness (DE): %12.8f\n', fval);
        fprintf(fileID, 'Geração do melhor (DE): %d\n', de_info.best_gen);

        fclose(fileID);

        disp('Log salvo com sucesso.');

        % Salvar a figura Data_and_displacement.fig
        file_name = 'Data_and_displacement.fig';
        full_file_path = fullfile(destination_path, file_name);
        savefig(fig, full_file_path);

        % Salvar a figura Route.fig
        route = [pR; z];
        plot(route(:, 1), route(:, 2), '-k');

        file_name = 'Route.fig';
        full_file_path = fullfile(destination_path, file_name);

        jpg_file_path = fullfile(destination_path, 'figure.jpg');
        saveas(fig, jpg_file_path);
        savefig(fig, full_file_path);

        disp('Figuras salvas com sucesso.');
        close(fig);
    end

    clear;
    clc;
    exit;
end


% =========================================================================
% Differential Evolution (DE) - implementação enxuta e robusta para 2D
% =========================================================================
function [best_x, best_f, info] = run_DE(func_fitness, lb, ub, NP, G, F, CR)
    % Entrada:
    %   func_fitness: handle que recebe x = [x y]
    %   lb, ub      : limites inferiores/superiores [1x2]
    %   NP          : tamanho da população
    %   G           : número máximo de gerações
    %   F           : fator diferencial (mutação)
    %   CR          : taxa de crossover
    %
    % Saída:
    %   best_x      : melhor solução encontrada
    %   best_f      : fitness correspondente
    %   info        : struct com informações úteis

    D = numel(lb); % aqui D = 2

    % ------------------------------
    % 1) Inicialização da população
    % ------------------------------
    pop = repmat(lb, NP, 1) + rand(NP, D) .* repmat((ub - lb), NP, 1);

    fit = zeros(NP, 1);
    for i = 1:NP
        fit(i) = func_fitness(pop(i, :));
    end

    [best_f, best_idx] = min(fit);
    best_x = pop(best_idx, :);
    best_gen = 0;

    % ---------------------------------------
    % 2) Loop evolutivo: mutate-crossover-select
    % ---------------------------------------
    for gen = 1:G
        for i = 1:NP

            % Seleciona r1, r2, r3 distintos e diferentes de i
            idx = randperm(NP);
            idx(idx == i) = [];     % remove i
            r1 = idx(1); r2 = idx(2); r3 = idx(3);

            % ------------------------------
            % Mutação diferencial (DE/rand/1)
            % v = x_r1 + F*(x_r2 - x_r3)
            % ------------------------------
            v = pop(r1, :) + F * (pop(r2, :) - pop(r3, :));

            % Tratamento de limites (clipping simples e estável)
            v = max(v, lb);
            v = min(v, ub);

            % ------------------------------
            % Crossover binomial
            % garante pelo menos 1 dimensão de v (jrand)
            % ------------------------------
            u = pop(i, :);
            jrand = randi(D);

            for j = 1:D
                if (rand <= CR) || (j == jrand)
                    u(j) = v(j);
                end
            end

            % Garante limites após crossover (por segurança)
            u = max(u, lb);
            u = min(u, ub);

            % ------------------------------
            % Seleção gulosa
            % ------------------------------
            fu = func_fitness(u);
            if fu <= fit(i)
                pop(i, :) = u;
                fit(i) = fu;

                % Atualiza melhor global
                if fu < best_f
                    best_f = fu;
                    best_x = u;
                    best_gen = gen;
                end
            end
        end
    end

    info.best_gen = best_gen;
end

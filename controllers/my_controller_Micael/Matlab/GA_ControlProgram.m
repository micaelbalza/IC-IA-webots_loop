%  Controle geral do programa, com etapas bem definidas:
%  Control_program()
%    |    |--> Automatic_reading() <--   <--   <--  <--  <--  <--
%    |    |--> Control_program(metaheuristic) <--> fitness()    /|\
%    |    |--> output                                            |
%    |    |--> Automatic_save()                                  |
%    |    |--> Wait  -->  -->  --> -->  -->  -->  -->   -->  --> |
%   \|/


function ControlProgram(destination_path, population_size, max_generations, elite_count, crossover_fraction, robot_Radius, Alpha)
    
    % Seed creation to ensure random program
    seed = round(sum(clock * 10000));
    rng(seed);

    population_size = str2double(population_size);
    max_generations = str2double(max_generations);
    elite_count = str2double(elite_count);
    crossover_fraction = str2double(crossover_fraction);


    options = gaoptimset(@ga);
    options.PopulationSize = population_size; 
    options.Generations = max_generations;
    options.EliteCount = elite_count;
    options.CrossoverFraction = crossover_fraction;

    robotRadius = 0.15;
    alpha = 1.4;
            
    tempos  = [];
    tempos2 = []; % Monitoramento da diversidade genética
    options.Vectorized = 'off'; % Desabilita 
    cont_m_displacement = 0; % displacement control assistant (for read and save txt file)
    pR = [];
    
    global Rd 
    Rd = 1;   % Radius (Rd) in meters --> sub radius of action (Rd in fitness too)
    
    client = tcpclient("localhost", 12345); % TCP connection

    while (1) 
        [K_points, m_displacement, pO_size, pR, final_objective, pDP, pO] = automatic_reading(cont_m_displacement, pR, destination_path);
        
        dist_max = 1;
        pDP_space = C_space(pDP, robotRadius, alpha); % Modification 1 - fitness function
        
        if (m_displacement == 0)
            [l, c] = size(pR);
            d = sqrt((pR(l, 1) - final_objective(1, 1))^2 + (pR(l, 2) - final_objective(1, 2))^2);
            % Final_distance - robot < 0.2 --> is it
            if (d < 0.2)
                break;
            end
        end

        % Func_fitness
        func_fitness = @(x) gJ(x, pDP_space, pO, pO_size, pR, final_objective, 1);
        [l, c] = size(pR);
        a = pR(l, 1);
        b = pR(l, 2);
        
        tic;
        t1 = cputime;
        % GA
        [z, fval, exitflag, output, population, scores] = ga(func_fitness, 2, [], [], [], [], [(a - Rd), (b - Rd)], [(a + Rd), (b + Rd)], [], options);

        t2 = cputime - t1;
        
        tempos2 = [toc, tempos2];
        tempos = [t2, tempos]; 
        
        [cont_m_displacement] = automatic_save(z, cont_m_displacement, destination_path);
        
        while true
            write(client, "read"); % send TCP flag to read txt file
            confirm = read(client, 8, 'char');
            if (confirm == "received")
                confirm = "";
                disp('Confirmado');
                break;
            end
            pause(0.5);
        end

        % Cria uma figura
        if ~exist('fig', 'var')
            fig = figure;
            hold on;
        end

        for c = 1:pO_size
            plot(pO(c, 1), pO(c, 2), "-o");
        end

        plot(final_objective(1), final_objective(2), 'diamond');
        plot(z(1), z(2), "*");
        plot(pR(m_displacement + 1, 1), pR(m_displacement + 1, 2), "square");
        %plot(population(:, 1), population(:, 2), 'x');
        plot(pDP_space(:, 1), pDP_space(:, 2), '-.', 'Color', 'blue');

        centers = [pR(m_displacement + 1, 1), pR(m_displacement + 1, 2)];

        d = sqrt((z(1) - final_objective(1, 1))^2 + (z(2) - final_objective(1, 2))^2);
        if (d < 0.2 || cont_m_displacement > 50)
            break;
        end
    end
    %disp("population")
    %disp(population)
    pR = [pR; z];

    if (cont_m_displacement < 50)
        route_distance = 0;
        max_distance = 0;

        for i = 1:m_displacement + 1
            aux = sqrt((pR(i + 1, 1) - pR(i, 1))^2 + (pR(i + 1, 2) - pR(i, 2))^2);
            route_distance = route_distance + aux;

            if aux > max_distance
                max_distance = aux;
            end
        end

        sum_tempos = sum(tempos);
        sum_tempos2 = sum(tempos2);

        file = fullfile(destination_path, 'log.txt');
        fileID = fopen(file, 'w');

        if fileID == -1
            error('Não foi possível abrir o arquivo.');
        end

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

        % Salvar a figura como JPG
        jpg_file_path = fullfile(destination_path, 'figure.jpg');
        saveas(fig, jpg_file_path);

        % Salvar a figura no formato .fig
        savefig(fig, full_file_path);

        disp('Figuras salvas com sucesso.');
        close(fig);
    end
    clear;
    clc;
    exit;
end




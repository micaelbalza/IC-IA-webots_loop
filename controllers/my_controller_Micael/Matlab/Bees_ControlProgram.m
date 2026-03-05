
% Controle geral do programa com o algoritmo das abelhas (Bees Algorithm)
function Bees_ControlProgram(destination_path, nScoutBees, max_generations, nEliteSites, nSelectedSites, nEliteBeesPerSite, nBeesPerSite, neighborhood_radius)

    seed = round(sum(clock * 10000));
    rng(seed);

    nScoutBees = str2double(nScoutBees);
    max_generations = str2double(max_generations);
    nEliteSites = str2double(nEliteSites);
    nSelectedSites = str2double(nSelectedSites);
    nEliteBeesPerSite = str2double(nEliteBeesPerSite);
    nBeesPerSite = str2double(nBeesPerSite);
    neighborhood_radius = str2double(neighborhood_radius);

    robotRadius = 0.15;
    alpha = 1.4;

    tempos = [];
    tempos2 = [];
    cont_m_displacement = 0;
    pR = [];

    global Rd
    Rd = 1;

    client = tcpclient("localhost", 12345);

    while (1)
        [K_points, m_displacement, pO_size, pR, final_objective, pDP, pO] = automatic_reading(cont_m_displacement, pR, destination_path);
        pDP_space = C_space(pDP, robotRadius, alpha);

        if (m_displacement == 0)
            d = norm(pR(end,:) - final_objective);
            if (d < 0.2)
                break;
            end
        end

        func_fitness = @(x) gJ(x, pDP_space, pO, pO_size, pR, final_objective, 1);
        a = pR(end,1);
        b = pR(end,2);
        lb = [(a - Rd), (b - Rd)];
        ub = [(a + Rd), (b + Rd)];

        tic;
        t1 = cputime;
        [best_sol, best_val] = run_BeesAlgorithm(func_fitness, lb, ub, nScoutBees, max_generations, ...
                                                 nEliteSites, nSelectedSites, nEliteBeesPerSite, ...
                                                 nBeesPerSite, neighborhood_radius);
        t2 = cputime - t1;

        tempos2 = [toc, tempos2];
        tempos = [t2, tempos];

        cont_m_displacement = automatic_save(best_sol, cont_m_displacement, destination_path);

        while true
            write(client, "read");
            confirm = read(client, 8, 'char');
            if (confirm == "received")
                break;
            end
            pause(0.5);
        end

        if ~exist('fig', 'var')
            fig = figure;
            hold on;
        end

        for c = 1:pO_size
            plot(pO(c, 1), pO(c, 2), "-o");
        end

        plot(final_objective(1), final_objective(2), 'diamond');
        plot(best_sol(1), best_sol(2), "*");
        plot(pR(m_displacement + 1, 1), pR(m_displacement + 1, 2), "square");
        plot(pDP_space(:, 1), pDP_space(:, 2), '-.', 'Color', 'blue');

        d = norm(best_sol - final_objective);
        if (d < 0.2 || cont_m_displacement > 50)
            break;
        end
    end

    pR = [pR; best_sol];

    if (cont_m_displacement < 50)
        route_distance = sum(vecnorm(diff(pR), 2, 2));
        sum_tempos = sum(tempos);
        sum_tempos2 = sum(tempos2);

        fileID = fopen(fullfile(destination_path, 'log.txt'), 'w');
        fprintf(fileID, 'Tempo de CPU:\n %12.8f\n\n', tempos);
        fprintf(fileID, 'Somatório CPU: %6.8f\n\n', sum_tempos);
        fprintf(fileID, 'Deslocamentos: %d\n\n', size(pR,1)-1);
        fprintf(fileID, 'Pontos do robô:\n');
        fprintf(fileID, '%6.8f %6.8f\n', pR');
        fprintf(fileID, '\nDistância percorrida: %6.8f\n\n', route_distance);
        fprintf(fileID, 'Tempo de relógio total: %6.8f\n\n', sum_tempos2);
        fprintf(fileID, 'Semente: %d\n', seed);
        fclose(fileID);

        plot(pR(:,1), pR(:,2), '-k');
        saveas(fig, fullfile(destination_path, 'figure.jpg'));
        savefig(fig, fullfile(destination_path, 'Data_and_displacement.fig'));
        savefig(fig, fullfile(destination_path, 'Route.fig'));
        close(fig);
    end

    clear;
    clc;
    exit;
end

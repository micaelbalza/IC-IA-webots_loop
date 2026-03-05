
function [best_sol, best_val] = run_BeesAlgorithm(fobj, lb, ub, nScoutBees, max_generations, ...
                                                  nEliteSites, nSelectedSites, ...
                                                  nEliteBeesPerSite, nBeesPerSite, neighborhood_radius)

    dim = length(lb);
    pop = repmat(lb, nScoutBees, 1) + rand(nScoutBees, dim) .* repmat(ub - lb, nScoutBees, 1);
    fitness = arrayfun(@(i) fobj(pop(i,:)), 1:nScoutBees)';
    [fitness, idx] = sort(fitness);
    pop = pop(idx, :);

    best_sol = pop(1,:);
    best_val = fitness(1);

    for gen = 1:max_generations
        new_pop = zeros(nScoutBees, dim);
        new_fitness = zeros(nScoutBees, 1);
        t = 1;

        % Sítios de elite
        for i = 1:nEliteSites
            site = pop(i,:);
            best_local = site;
            best_local_val = fobj(site);
            for j = 1:nEliteBeesPerSite
                candidate = site + neighborhood_radius * (2*rand(1, dim)-1);
                candidate = max(min(candidate, ub), lb);
                val = fobj(candidate);
                if val < best_local_val
                    best_local = candidate;
                    best_local_val = val;
                end
            end
            new_pop(t,:) = best_local;
            new_fitness(t) = best_local_val;
            t = t + 1;
        end

        % Sítios selecionados (não elite)
        for i = nEliteSites+1:nSelectedSites
            site = pop(i,:);
            best_local = site;
            best_local_val = fobj(site);
            for j = 1:nBeesPerSite
                candidate = site + neighborhood_radius * (2*rand(1, dim)-1);
                candidate = max(min(candidate, ub), lb);
                val = fobj(candidate);
                if val < best_local_val
                    best_local = candidate;
                    best_local_val = val;
                end
            end
            new_pop(t,:) = best_local;
            new_fitness(t) = best_local_val;
            t = t + 1;
        end

        % Abelhas exploradoras restantes
        while t <= nScoutBees
            scout = lb + rand(1, dim) .* (ub - lb);
            new_pop(t,:) = scout;
            new_fitness(t) = fobj(scout);
            t = t + 1;
        end

        % Atualização da população
        pop = new_pop;
        fitness = new_fitness;
        [fitness, idx] = sort(fitness);
        pop = pop(idx, :);

        if fitness(1) < best_val
            best_val = fitness(1);
            best_sol = pop(1,:);
        end
    end
end

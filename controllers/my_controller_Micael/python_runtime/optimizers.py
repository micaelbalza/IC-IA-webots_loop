import math
import random


def run_woa(objective, lb, ub, search_agents, max_iter, spiral_coefficient):
    dim = len(lb)
    population = [_random_point(lb, ub) for _ in range(search_agents)]
    fitness = [objective(point) for point in population]
    leader_index = min(range(search_agents), key=lambda idx: fitness[idx])
    leader_position = population[leader_index][:]
    leader_score = fitness[leader_index]

    for iteration in range(1, max_iter + 1):
        a = 2 - iteration * (2 / max_iter)
        for index in range(search_agents):
            r1 = random.random()
            r2 = random.random()
            a_term = 2 * a * r1 - a
            c_term = 2 * r2
            l_term = random.random() * 2 - 1
            p_term = random.random()

            if p_term < 0.5:
                if abs(a_term) >= 1:
                    random_index = random.randrange(search_agents)
                    distance = [
                        abs(c_term * population[random_index][dim_index] - population[index][dim_index])
                        for dim_index in range(dim)
                    ]
                    candidate = [
                        population[random_index][dim_index] - a_term * distance[dim_index]
                        for dim_index in range(dim)
                    ]
                else:
                    distance = [
                        abs(c_term * leader_position[dim_index] - population[index][dim_index])
                        for dim_index in range(dim)
                    ]
                    candidate = [
                        leader_position[dim_index] - a_term * distance[dim_index]
                        for dim_index in range(dim)
                    ]
            else:
                distance = [
                    abs(leader_position[dim_index] - population[index][dim_index])
                    for dim_index in range(dim)
                ]
                candidate = [
                    distance[dim_index] * math.exp(spiral_coefficient * l_term) * math.cos(2 * math.pi * l_term)
                    + leader_position[dim_index]
                    for dim_index in range(dim)
                ]

            population[index] = _clip(candidate, lb, ub)
            candidate_fitness = objective(population[index])
            if candidate_fitness < leader_score:
                leader_score = candidate_fitness
                leader_position = population[index][:]

    return leader_position, leader_score, {"algorithm": "WOA"}


def run_de(objective, lb, ub, population_size, generations, differential_weight, crossover_rate):
    dim = len(lb)
    population = [_random_point(lb, ub) for _ in range(population_size)]
    fitness = [objective(point) for point in population]

    best_index = min(range(population_size), key=lambda idx: fitness[idx])
    best_solution = population[best_index][:]
    best_fitness = fitness[best_index]
    best_generation = 0

    for generation in range(1, generations + 1):
        for index in range(population_size):
            available = [candidate for candidate in range(population_size) if candidate != index]
            r1, r2, r3 = random.sample(available, 3)
            mutant = [
                population[r1][dim_index] + differential_weight * (population[r2][dim_index] - population[r3][dim_index])
                for dim_index in range(dim)
            ]
            mutant = _clip(mutant, lb, ub)

            trial = population[index][:]
            mandatory_dimension = random.randrange(dim)
            for dim_index in range(dim):
                if random.random() <= crossover_rate or dim_index == mandatory_dimension:
                    trial[dim_index] = mutant[dim_index]

            trial_fitness = objective(trial)
            if trial_fitness <= fitness[index]:
                population[index] = trial
                fitness[index] = trial_fitness

                if trial_fitness < best_fitness:
                    best_solution = trial[:]
                    best_fitness = trial_fitness
                    best_generation = generation

    return best_solution, best_fitness, {
        "algorithm": "DE",
        "best_gen": best_generation,
        "F": differential_weight,
        "CR": crossover_rate,
    }


def run_bees(
    objective,
    lb,
    ub,
    scout_bees,
    generations,
    elite_sites,
    selected_sites,
    elite_bees_per_site,
    bees_per_site,
    neighborhood_radius,
):
    dim = len(lb)
    population = [_random_point(lb, ub) for _ in range(scout_bees)]
    fitness = [objective(point) for point in population]
    population = [point for _, point in sorted(zip(fitness, population), key=lambda item: item[0])]
    fitness = sorted(fitness)

    best_solution = population[0][:]
    best_fitness = fitness[0]

    for _ in range(generations):
        new_population = []
        new_fitness = []

        for index in range(elite_sites):
            site_solution, site_fitness = _neighborhood_search(
                objective, population[index], elite_bees_per_site, neighborhood_radius, lb, ub, dim
            )
            new_population.append(site_solution)
            new_fitness.append(site_fitness)

        for index in range(elite_sites, selected_sites):
            site_solution, site_fitness = _neighborhood_search(
                objective, population[index], bees_per_site, neighborhood_radius, lb, ub, dim
            )
            new_population.append(site_solution)
            new_fitness.append(site_fitness)

        while len(new_population) < scout_bees:
            scout = _random_point(lb, ub)
            new_population.append(scout)
            new_fitness.append(objective(scout))

        population = [point for _, point in sorted(zip(new_fitness, new_population), key=lambda item: item[0])]
        fitness = sorted(new_fitness)
        if fitness[0] < best_fitness:
            best_fitness = fitness[0]
            best_solution = population[0][:]

    return best_solution, best_fitness, {"algorithm": "BA"}


def run_pso(objective, lb, ub, swarm_size, max_iterations, self_weight, social_weight):
    dim = len(lb)
    inertia = 0.7
    positions = [_random_point(lb, ub) for _ in range(swarm_size)]
    velocities = [[0.0] * dim for _ in range(swarm_size)]
    personal_best_positions = [position[:] for position in positions]
    personal_best_scores = [objective(position) for position in positions]
    global_best_index = min(range(swarm_size), key=lambda idx: personal_best_scores[idx])
    global_best_position = personal_best_positions[global_best_index][:]
    global_best_score = personal_best_scores[global_best_index]

    for _ in range(max_iterations):
        for index in range(swarm_size):
            for dim_index in range(dim):
                r1 = random.random()
                r2 = random.random()
                velocities[index][dim_index] = (
                    inertia * velocities[index][dim_index]
                    + self_weight * r1 * (personal_best_positions[index][dim_index] - positions[index][dim_index])
                    + social_weight * r2 * (global_best_position[dim_index] - positions[index][dim_index])
                )
                positions[index][dim_index] += velocities[index][dim_index]

            positions[index] = _clip(positions[index], lb, ub)
            score = objective(positions[index])
            if score < personal_best_scores[index]:
                personal_best_scores[index] = score
                personal_best_positions[index] = positions[index][:]
                if score < global_best_score:
                    global_best_score = score
                    global_best_position = positions[index][:]

    return global_best_position, global_best_score, {"algorithm": "PSO"}


def run_ga(objective, lb, ub, population_size, generations, elite_count, crossover_fraction):
    dim = len(lb)
    mutation_rate = 0.1
    population = [_random_point(lb, ub) for _ in range(population_size)]

    best_solution = None
    best_fitness = float("inf")

    for _ in range(generations):
        population.sort(key=objective)
        fitness_values = [objective(candidate) for candidate in population]
        if fitness_values[0] < best_fitness:
            best_fitness = fitness_values[0]
            best_solution = population[0][:]

        next_population = [candidate[:] for candidate in population[:elite_count]]

        while len(next_population) < population_size:
            parent1 = _tournament_selection(population, objective)
            parent2 = _tournament_selection(population, objective)
            child1 = parent1[:]
            child2 = parent2[:]

            if random.random() <= crossover_fraction:
                crossover_point = random.randrange(1, dim)
                child1 = parent1[:crossover_point] + parent2[crossover_point:]
                child2 = parent2[:crossover_point] + parent1[crossover_point:]

            child1 = _mutate(child1, lb, ub, mutation_rate)
            child2 = _mutate(child2, lb, ub, mutation_rate)
            next_population.append(_clip(child1, lb, ub))
            if len(next_population) < population_size:
                next_population.append(_clip(child2, lb, ub))

        population = next_population

    if best_solution is None:
        population.sort(key=objective)
        best_solution = population[0][:]
        best_fitness = objective(best_solution)

    return best_solution, best_fitness, {"algorithm": "GA"}


def _neighborhood_search(objective, site, bees_count, neighborhood_radius, lb, ub, dim):
    best_local = site[:]
    best_local_fitness = objective(site)
    for _ in range(bees_count):
        candidate = [
            site[dim_index] + neighborhood_radius * (2 * random.random() - 1)
            for dim_index in range(dim)
        ]
        candidate = _clip(candidate, lb, ub)
        score = objective(candidate)
        if score < best_local_fitness:
            best_local = candidate
            best_local_fitness = score
    return best_local, best_local_fitness


def _tournament_selection(population, objective, size=3):
    contestants = random.sample(population, min(size, len(population)))
    contestants.sort(key=objective)
    return contestants[0][:]


def _mutate(candidate, lb, ub, mutation_rate):
    for index in range(len(candidate)):
        if random.random() < mutation_rate:
            candidate[index] = random.uniform(lb[index], ub[index])
    return candidate


def _random_point(lb, ub):
    return [random.uniform(lower, upper) for lower, upper in zip(lb, ub)]


def _clip(candidate, lb, ub):
    return [max(lower, min(value, upper)) for value, lower, upper in zip(candidate, lb, ub)]

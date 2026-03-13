import math
import random
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.append(str(Path(__file__).resolve().parent.parent))
    from python_runtime.fitness import g_j
    from python_runtime.geometry import c_space
    from python_runtime.io_bridge import (
        MatlabCompatibleClient,
        automatic_reading,
        automatic_save,
        save_log,
        save_plot_artifacts,
    )
    from python_runtime.optimizers import run_bees, run_de, run_ga, run_pso, run_woa
else:
    from .fitness import g_j
    from .geometry import c_space
    from .io_bridge import (
        MatlabCompatibleClient,
        automatic_reading,
        automatic_save,
        save_log,
        save_plot_artifacts,
    )
    from .optimizers import run_bees, run_de, run_ga, run_pso, run_woa


ROBOT_RADIUS = 0.15
ALPHA = 1.4
RD = 1.0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) < 2:
        raise SystemExit("Usage: control_program.py <METAHEURISTIC> <destination_path> [params...]")

    metaheuristic = argv[0].upper()
    destination_path = argv[1]
    raw_parameters = argv[2:]

    seed = round(time.time() * 10000)
    random.seed(seed)

    client = MatlabCompatibleClient()
    cpu_times = []
    wall_times = []
    pr = []
    cont_m_displacement = 0
    z = None
    optimizer_info = {}
    trajectory_debug = []

    try:
        while True:
            _, m_displacement, po_size, pr, final_objective, pdp, po = automatic_reading(
                cont_m_displacement, pr, destination_path
            )
            pdp_space = c_space(pdp, ROBOT_RADIUS, ALPHA)

            if m_displacement == 0 and math.dist(pr[-1], final_objective) < 0.2:
                break

            current_position = pr[-1]
            lb = [current_position[0] - RD, current_position[1] - RD]
            ub = [current_position[0] + RD, current_position[1] + RD]

            objective = lambda candidate: g_j(candidate, pdp_space, po, po_size, pr, final_objective)

            wall_start = time.perf_counter()
            cpu_start = time.process_time()
            z, best_fitness, optimizer_info = run_optimizer(metaheuristic, objective, lb, ub, raw_parameters)
            cpu_times.insert(0, time.process_time() - cpu_start)
            wall_times.insert(0, time.perf_counter() - wall_start)

            cont_m_displacement = automatic_save(z, cont_m_displacement, destination_path)
            client.notify_read()

            trajectory_debug.append(
                {
                    "m_displacement": m_displacement,
                    "current_position": current_position,
                    "chosen_point": z,
                    "final_objective": final_objective,
                    "best_fitness": best_fitness,
                    "pdp_space": pdp_space,
                    "obstacles": po,
                }
            )

            if math.dist(z, final_objective) < 0.2 or cont_m_displacement > 50:
                break
    finally:
        client.close()

    route = [point[:] for point in pr]
    if z is not None:
        route.append(z[:])

    if cont_m_displacement < 50 and route:
        route_distance = sum(math.dist(route[index + 1], route[index]) for index in range(len(route) - 1))
        save_log(
            destination_path,
            {
                "cpu_times": cpu_times,
                "wall_times": wall_times,
                "displacements": len(route) - 1,
                "route": route,
                "route_distance": route_distance,
                "seed": seed,
                "optimizer_info": optimizer_info,
            },
        )
        save_plot_artifacts(
            destination_path,
            {
                "route": route,
                "iterations": trajectory_debug,
                "seed": seed,
                "optimizer": optimizer_info,
                "note": "Python placeholder artifact replacing MATLAB .fig output.",
            },
        )


def run_optimizer(metaheuristic, objective, lb, ub, raw_parameters):
    if metaheuristic == "WOA":
        population_size = int(raw_parameters[0])
        max_generations = int(raw_parameters[1])
        spiral_coefficient = float(raw_parameters[2])
        return run_woa(objective, lb, ub, population_size, max_generations, spiral_coefficient)

    if metaheuristic == "DE":
        population_size = int(raw_parameters[0])
        max_generations = int(raw_parameters[1])
        differential_weight = float(raw_parameters[2])
        crossover_rate = float(raw_parameters[3])
        return run_de(objective, lb, ub, population_size, max_generations, differential_weight, crossover_rate)

    if metaheuristic == "BA":
        return run_bees(
            objective,
            lb,
            ub,
            int(raw_parameters[0]),
            int(raw_parameters[1]),
            int(raw_parameters[2]),
            int(raw_parameters[3]),
            int(raw_parameters[4]),
            int(raw_parameters[5]),
            float(raw_parameters[6]),
        )

    if metaheuristic == "PSO":
        population_size = int(raw_parameters[0])
        max_generations = int(raw_parameters[1])
        self_weight = float(raw_parameters[2])
        social_weight = float(raw_parameters[3])
        return run_pso(objective, lb, ub, population_size, max_generations, self_weight, social_weight)

    if metaheuristic == "GA":
        population_size = int(raw_parameters[0])
        max_generations = int(raw_parameters[1])
        elite_count = int(raw_parameters[2])
        crossover_fraction = float(raw_parameters[3])
        return run_ga(objective, lb, ub, population_size, max_generations, elite_count, crossover_fraction)

    raise ValueError(f"Unsupported metaheuristic: {metaheuristic}")


if __name__ == "__main__":
    main()

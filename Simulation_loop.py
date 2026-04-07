import subprocess
import os
import time
import json
import signal
from typing import Optional


# =============================================================================
# 1) UTILITÁRIOS ROBUSTOS PARA INICIAR E ENCERRAR PROCESSOS (LINUX)
#
# Objetivo:
# - Evitar que Webots/programa de controle deixem processos filhos órfãos quando ocorre erro.
# - Garantir que ao final de cada simulação (sucesso, erro ou timeout) tudo seja
#   encerrado e o sistema fique "limpo" para a próxima repetição.
#
# Estratégia:
# - Iniciar cada processo em uma NOVA SESSÃO (process group) via start_new_session=True
# - Encerrar o GRUPO inteiro (processo + filhos) com SIGTERM e, se necessário, SIGKILL
# =============================================================================
def start_process(cmd, env=None, cwd=None):
    """
    Inicia o processo em uma nova sessão (process group).

    - start_new_session=True cria um novo grupo de processos.
    - Isso permite matar o processo e TODOS os filhos com os.killpg(pid, sinal).
    - stdout/stderr são descartados para evitar travamentos por buffers.
      (Se quiser depurar, redirecione para arquivo).
    """
    return subprocess.Popen(
        cmd,
        env=env,
        cwd=cwd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True  # <- ESSENCIAL (Linux)
    )


def terminate_process_tree(p: Optional[subprocess.Popen], timeout_s: float = 20.0):
    """
    Encerra um processo e TODOS os processos filhos do mesmo grupo (process group).

    Fluxo:
      1) SIGTERM no grupo  -> pede encerramento gracioso
      2) espera timeout_s  -> dá tempo do app fechar corretamente
      3) SIGKILL no grupo  -> força encerramento se ainda estiver vivo
    """
    if p is None:
        return

    # Se já terminou, não faz nada
    if p.poll() is not None:
        return

    # 1) Encerramento gracioso
    try:
        os.killpg(p.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    except Exception:
        pass

    # 2) Aguarda encerrar
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if p.poll() is not None:
            return
        time.sleep(0.2)

    # 3) Força encerramento
    try:
        os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    except Exception:
        pass


# =============================================================================
# 2) UTILITÁRIOS DE ARQUIVOS/PASTAS (MANTIDOS)
# =============================================================================
def criar_pasta_principal(nome_pasta_principal, path_to_save_simulation_files):
    """
    Cria a pasta principal que agrupa um tipo de simulação, por exemplo:
        Env1-DE-P20-G30-F0.6-CR0.9/
    Retorna False se já existir, para evitar sobrescrever.
    """
    caminho_pasta_principal = os.path.join(path_to_save_simulation_files, nome_pasta_principal)
    try:
        os.makedirs(caminho_pasta_principal)
    except FileExistsError:
        print(f"The folder {caminho_pasta_principal} already exists.")
        print("\n Nothing will be done! \n")
        return False


def clean_folder(caminho_pasta):
    """
    Remove arquivos soltos dentro da pasta (não remove subpastas).
    Útil para evitar que a simulação atual use arquivos antigos.
    """
    for arquivo in os.listdir(caminho_pasta):
        arquivo_path = os.path.join(caminho_pasta, arquivo)
        try:
            if os.path.isfile(arquivo_path):
                os.unlink(arquivo_path)
        except Exception as e:
            print(f"Erro ao excluir {arquivo_path}: {e}")


# =============================================================================
# 3) MAIN
#
# Fluxo geral:
#   - Lê simulationSetup.json
#   - Para cada setup:
#       - Monta simulation_name e parâmetros do programa de controle
#       - Cria pasta principal
#       - Repete 'simulation_repeat' vezes:
#           - Cria pasta da repetição (request/response)
#           - Inicia Webots
#           - Inicia o programa de controle
#           - Aguarda terminar por:
#               (a) programa de controle encerrar (sucesso/erro) -> PARA AQUI (break)
#               (b) Timeout                          -> PARA AQUI (break)
#           - SEMPRE: encerra processos (programa de controle/Webots + filhos)
#           - SEMPRE: espera sleep(130) antes da próxima simulação
#       - Escreve log.txt ao final do setup
# =============================================================================
if __name__ == "__main__":

    # -------------------------------------------------------------------------
    # Leitura do arquivo JSON com todos os setups
    # -------------------------------------------------------------------------
    with open('simulationSetup.json', 'r') as arquivo:
        simulation_setup = json.load(arquivo)

    # -------------------------------------------------------------------------
    # Loop por setup (tipo de simulação)
    # -------------------------------------------------------------------------
    for index, actual_setup in enumerate(simulation_setup, start=1):

        # --------------------- parâmetros gerais -----------------------------
        metaheuristic = actual_setup["metaheuristic"]

        robot_radius = float(actual_setup["robot_radius"])
        alpha = float(actual_setup["alpha"])

        environment_file = actual_setup["environment"]
        path_to_save_simulation_files = actual_setup["path_to_save_simulation_files"]
        metaheuristic_runner_path = actual_setup["metaheuristic_runner_path"]

        simulation_repeat = int(actual_setup["simulation_repeat"])
        simulation_time = float(actual_setup["average_time_in_each_simulation"])
        Webots_initiation_time = float(actual_setup["webots_initiation_time"])
        control_program_initiation_time = float(actual_setup["control_program_initiation_time"])
        end_point_x = float(actual_setup["end_point_x"])
        end_point_y = float(actual_setup["end_point_y"])

        # ------------------------- montar nome e parâmetros -------------------
        simulation_name = None

        if metaheuristic == 'PSO':
            population_size = int(actual_setup["population_size"])
            max_generations = int(actual_setup["max_generations"])
            self_adjustment_weight = float(actual_setup["self_adjustment_weight"])
            social_adjustment_weight = float(actual_setup["social_adjustment_weight"])

            simulation_name = (
                os.path.splitext(os.path.basename(environment_file))[0] + "-" +
                metaheuristic + "-P" + str(population_size) +
                "-G" + str(max_generations) +
                "-C" + str(self_adjustment_weight) +
                "-S" + str(social_adjustment_weight) + "/"
            )

        elif metaheuristic == "GA":
            population_size = int(actual_setup["population_size"])
            max_generations = int(actual_setup["max_generations"])
            elite_count = int(actual_setup["elite_count"])
            crossover_fraction = float(actual_setup["crossover_fraction"])

            simulation_name = (
                os.path.splitext(os.path.basename(environment_file))[0] + "-" +
                metaheuristic + "-P" + str(population_size) +
                "-G" + str(max_generations) +
                "-E" + str(elite_count) +
                "-C" + str(crossover_fraction) + "/"
            )

        elif metaheuristic == "WOA":
            population_size = int(actual_setup["population_size"])
            max_generations = int(actual_setup["max_generations"])
            spiral_coefficient = float(actual_setup["spiral_coefficient"])

            simulation_name = (
                os.path.splitext(os.path.basename(environment_file))[0] + "-" +
                metaheuristic + "-P" + str(population_size) +
                "-G" + str(max_generations) +
                "-SC" + str(spiral_coefficient) + "/"
            )

        elif metaheuristic == "BA":
            nScoutBees = int(actual_setup["nScoutBees"])
            max_generations = int(actual_setup["max_generations"])
            neighborhood_radius = float(actual_setup["neighborhood_radius"])
            nEliteSites = int(actual_setup["nEliteSites"])
            nSelectedSites = int(actual_setup["nSelectedSites"])
            nEliteBeesPerSite = int(actual_setup["nEliteBeesPerSite"])
            nBeesPerSite = int(actual_setup["nBeesPerSite"])

            simulation_name = (
                os.path.splitext(os.path.basename(environment_file))[0] + "-" +
                metaheuristic + "-P" + str(nScoutBees) +
                "-G" + str(max_generations) +
                "-NR" + str(neighborhood_radius) + "/"
            )

        elif metaheuristic == "DE":
            population_size = int(actual_setup["population_size"])
            max_generations = int(actual_setup["max_generations"])
            F = float(actual_setup["F"])
            CR = float(actual_setup["CR"])

            simulation_name = (
                os.path.splitext(os.path.basename(environment_file))[0] + "-" +
                metaheuristic + "-P" + str(population_size) +
                "-G" + str(max_generations) +
                "-F" + str(F) +
                "-CR" + str(CR) + "/"
            )

        else:
            print("Error - Reading not implemented - metaheuristic fail")
            continue

        # ------------------------- cria pasta principal -----------------------
        erro = criar_pasta_principal(simulation_name, path_to_save_simulation_files)
        if erro is False:
            continue

        # ------------------------- contadores para log ------------------------
        time_limit_count = 0
        process_termination_count = 0
        simulations_time_limit = []
        simulations_process_termination = []
        simulations_control_program_success = []
        simulations_control_program_error = []

        # ---------------------------------------------------------------------
        # Loop de repetições
        # ---------------------------------------------------------------------
        for current_simulation_number in range(1, simulation_repeat + 1):

            current_simulation_number_str = str(current_simulation_number).zfill(2)

            destination_folder = (
                path_to_save_simulation_files +
                simulation_name +
                current_simulation_number_str + "/"
            )

            # Preparar pastas desta repetição
            os.makedirs(destination_folder, exist_ok=True)
            clean_folder(destination_folder)

            os.makedirs(destination_folder + "request/", exist_ok=True)
            os.makedirs(destination_folder + "response/", exist_ok=True)

            processo_webots = None
            control_program_process = None

            # flags de término (para log e debug)
            ended_by_timeout = False
            control_program_returncode = None

            try:
                # =============================================================
                # A) Inicia Webots
                # =============================================================
                env = os.environ.copy()
                env["DESTINATION_FOLDER"] = destination_folder
                env["END_POINT"] = f"{end_point_x},{end_point_y}"

                time.sleep(1)

                webots_command = ["xvfb-run", "-a", "webots", "--batch", "--no-rendering" ,  "--minimize", "--mode=fast" , environment_file] #, , , "--no-rendering" ,  "--minimize"
                processo_webots = start_process(webots_command, env=env)

                time.sleep(Webots_initiation_time)
                print("Webots has been started.")

                # =============================================================
                # B) Inicia o programa de controle
                # =============================================================
                work_path = destination_folder

                python_control_program = metaheuristic_runner_path

                if metaheuristic == 'PSO':
                    control_command = [
                        "python3",
                        python_control_program,
                        metaheuristic,
                        work_path,
                        str(population_size),
                        str(max_generations),
                        str(self_adjustment_weight),
                        str(social_adjustment_weight),
                    ]
                elif metaheuristic == "GA":
                    control_command = [
                        "python3",
                        python_control_program,
                        metaheuristic,
                        work_path,
                        str(population_size),
                        str(max_generations),
                        str(elite_count),
                        str(crossover_fraction),
                    ]
                elif metaheuristic == "WOA":
                    control_command = [
                        "python3",
                        python_control_program,
                        metaheuristic,
                        work_path,
                        str(population_size),
                        str(max_generations),
                        str(spiral_coefficient),
                    ]
                elif metaheuristic == "BA":
                    control_command = [
                        "python3",
                        python_control_program,
                        metaheuristic,
                        work_path,
                        str(nScoutBees),
                        str(max_generations),
                        str(nEliteSites),
                        str(nSelectedSites),
                        str(nEliteBeesPerSite),
                        str(nBeesPerSite),
                        str(neighborhood_radius),
                    ]
                elif metaheuristic == "DE":
                    control_command = [
                        "python3",
                        python_control_program,
                        metaheuristic,
                        work_path,
                        str(population_size),
                        str(max_generations),
                        str(F),
                        str(CR),
                    ]
                else:
                    raise RuntimeError("Metaheuristic not implemented for control_command.")

                control_program_process = start_process(control_command)

                print("work_path")
                print(work_path)
                
                time.sleep(60) # 5

                time.sleep(control_program_initiation_time)
                print("Python control program has been started.")

                # =============================================================
                # C) Aguarda finalizar por PROGRAMA DE CONTROLE ENCERRAR ou TIMEOUT
                #
                # Correção importante:
                # - Se o programa de controle terminar, nós SAÍMOS do loop (break) imediatamente.
                # - Isso evita que "dê timeout" mesmo quando a simulação concluiu.
                # =============================================================
                t0 = time.time()

                while True:
                    # 1) Checar se o programa de controle terminou
                    control_program_returncode = control_program_process.poll()
                    if control_program_returncode is not None:
                        process_termination_count += 1
                        simulations_process_termination.append(current_simulation_number_str)

                        if control_program_returncode == 0:
                            simulations_control_program_success.append(current_simulation_number_str)
                            print("Control program terminou com sucesso (returncode=0).")
                        else:
                            simulations_control_program_error.append(current_simulation_number_str)
                            print(f"Control program terminou com erro (returncode={control_program_returncode}).")
                        break

                    # 2) Checar timeout
                    if time.time() - t0 > simulation_time:
                        ended_by_timeout = True
                        time_limit_count += 1
                        simulations_time_limit.append(current_simulation_number_str)
                        print("Timeout has been reached")
                        break

                    time.sleep(0.2)
                time.sleep(60) # 5

            finally:
                # =============================================================
                # D) Encerramento garantido (SEMPRE)
                # - Mesmo se o programa de controle terminou sozinho (sucesso/erro),
                #   ainda garantimos que Webots e filhos sejam encerrados.
                # - Se deu timeout, este bloco é o responsável por matar tudo.
                # =============================================================
                terminate_process_tree(control_program_process, timeout_s=25)
                terminate_process_tree(processo_webots, timeout_s=25)

                # Fallback para controller específico (caso raro fique órfão)
                project_root = os.path.dirname(os.path.abspath(__file__))
                controller_binary = os.path.join(
                    project_root,
                    "controllers",
                    "my_controller_Micael",
                    "my_controller_Micael",
                )
                subprocess.run(
                    ['pkill', '-f', controller_binary],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )

                # Mensagem final desta repetição
                if ended_by_timeout:
                    print(f"\n The simulation {simulation_name}{current_simulation_number_str} finished (TIMEOUT)\n")
                else:
                    # terminou pelo programa de controle encerrar
                    print(f"\n The simulation {simulation_name}{current_simulation_number_str} finished (CONTROL_PROGRAM_END rc={control_program_returncode})\n")

                # Mantido conforme solicitado
                time.sleep(130)

        # ---------------------------------------------------------------------
        # Escrita do log ao final deste setup
        # ---------------------------------------------------------------------
        log_filename = os.path.join(path_to_save_simulation_files, simulation_name, "log.txt")
        with open(log_filename, "w") as log_file:
            log_file.write("=== SUMMARY ===\n\n")

            log_file.write("Number of simulations terminated due to time limit (TIMEOUT): {}\n".format(time_limit_count))
            log_file.write("Simulations terminated due to time limit (TIMEOUT):\n")
            for sim in simulations_time_limit:
                log_file.write(sim + "\n")

            log_file.write("\n")

            log_file.write("Number of simulations where control program ended: {}\n".format(process_termination_count))
            log_file.write("Simulations where control program ended:\n")
            for sim in simulations_process_termination:
                log_file.write(sim + "\n")

            log_file.write("\n")

            log_file.write("Control program ended with SUCCESS (returncode=0): {}\n".format(len(simulations_control_program_success)))
            log_file.write("Simulations with control program success:\n")
            for sim in simulations_control_program_success:
                log_file.write(sim + "\n")

            log_file.write("\n")

            log_file.write("Control program ended with ERROR (returncode!=0): {}\n".format(len(simulations_control_program_error)))
            log_file.write("Simulations with control program error:\n")
            for sim in simulations_control_program_error:
                log_file.write(sim + "\n")

    print("All simulations are over - the program has ended")

import subprocess
import os
import time
import json
import signal
import re
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
def start_process(cmd, env=None, cwd=None, stdout_path=None, stderr_path=None):
    """
    Inicia o processo em uma nova sessão (process group).

    - start_new_session=True cria um novo grupo de processos.
    - Isso permite matar o processo e TODOS os filhos com os.killpg(pid, sinal).
    - stdout/stderr são descartados para evitar travamentos por buffers.
      (Se quiser depurar, redirecione para arquivo).
    """
    stdout_target = subprocess.DEVNULL
    stderr_target = subprocess.DEVNULL
    opened_streams = []

    if stdout_path:
        stdout_target = open(stdout_path, "ab")
        opened_streams.append(stdout_target)
    if stderr_path:
        stderr_target = open(stderr_path, "ab")
        opened_streams.append(stderr_target)

    process = subprocess.Popen(
        cmd,
        env=env,
        cwd=cwd,
        stdout=stdout_target,
        stderr=stderr_target,
        start_new_session=True  # <- ESSENCIAL (Linux)
    )
    process._codex_opened_streams = opened_streams
    return process


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

    def close_streams():
        for stream in getattr(p, "_codex_opened_streams", []):
            try:
                stream.close()
            except Exception:
                pass

    # Se já terminou, não faz nada
    if p.poll() is not None:
        close_streams()
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
            close_streams()
            return
        time.sleep(0.2)

    # 3) Força encerramento
    try:
        os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    except Exception:
        pass
    close_streams()


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


def safe_remove_file(file_path):
    if not file_path:
        return
    try:
        if os.path.isfile(file_path):
            os.remove(file_path)
    except OSError:
        pass


def restore_file_contents(file_path, original_contents):
    if original_contents is None:
        return
    try:
        with open(file_path, "w", encoding="utf-8") as restored_file:
            restored_file.write(original_contents)
    except OSError:
        pass


def parse_optional_noise_config(actual_setup):
    """
    Retorna o bloco opcional de configuração de ruído do setup.
    """
    sensor_noise = actual_setup.get("sensor_noise")
    if not sensor_noise:
        return None
    if not isinstance(sensor_noise, dict):
        raise ValueError("The optional 'sensor_noise' field must be an object in simulationSetup.json.")
    return sensor_noise


def parse_optional_noise_label(actual_setup, sensor_noise):
    """
    Retorna um sufixo opcional de nome para experimentos com ruído.
    """
    noise_label = actual_setup.get("sensor_noise_label")
    if noise_label is None and sensor_noise:
        noise_label = "NOISE"
    if noise_label is None:
        return ""

    if not isinstance(noise_label, str):
        noise_label = str(noise_label)

    sanitized_label = re.sub(r"[^A-Za-z0-9._-]+", "_", noise_label.strip())
    if not sanitized_label:
        return ""
    return "-" + sanitized_label


def prepare_environment_with_noise(environment_file, destination_folder, sensor_noise):
    """
    Gera uma cópia temporária do mundo .wbt com a configuração opcional de ruído.

    O arquivo original nunca é alterado. A cópia fica dentro da pasta da simulação
    corrente para facilitar rastreabilidade e uso dentro do container.
    """
    if not sensor_noise:
        return environment_file, None

    with open(environment_file, "r", encoding="utf-8") as world_file:
        original_world_text = world_file.read()
    world_text = original_world_text

    world_random_seed = sensor_noise.get("world_random_seed", sensor_noise.get("random_seed"))
    if world_random_seed is not None:
        world_text = update_named_block_fields(
            world_text,
            "WorldInfo",
            {"randomSeed": world_random_seed},
            apply_to_all=False,
        )

    gps_settings = normalize_noise_fields(
        sensor_noise.get("gps"),
        {
            "accuracy": "accuracy",
            "noiseCorrelation": "noiseCorrelation",
            "noise_correlation": "noiseCorrelation",
            "resolution": "resolution",
            "speedNoise": "speedNoise",
            "speed_noise": "speedNoise",
            "speedResolution": "speedResolution",
            "speed_resolution": "speedResolution",
        },
    )
    if gps_settings:
        world_text = update_named_block_fields(world_text, "GPS", gps_settings, apply_to_all=False)

    compass_settings = normalize_noise_fields(
        sensor_noise.get("compass"),
        {
            "lookupTable": "lookupTable",
            "lookup_table": "lookupTable",
            "resolution": "resolution",
            "xAxis": "xAxis",
            "x_axis": "xAxis",
            "yAxis": "yAxis",
            "y_axis": "yAxis",
            "zAxis": "zAxis",
            "z_axis": "zAxis",
        },
    )
    if compass_settings:
        if "lookupTable" in compass_settings:
            compass_settings["lookupTable"] = normalize_compass_lookup_table(compass_settings["lookupTable"])
        world_text = update_named_block_fields(world_text, "Compass", compass_settings, apply_to_all=False)

    distance_sensor_settings = normalize_noise_fields(
        sensor_noise.get("distance_sensor"),
        {
            "lookupTable": "lookupTable",
            "lookup_table": "lookupTable",
            "resolution": "resolution",
            "aperture": "aperture",
            "gaussianWidth": "gaussianWidth",
            "gaussian_width": "gaussianWidth",
            "type": "type",
            "redColorSensitivity": "redColorSensitivity",
            "red_color_sensitivity": "redColorSensitivity",
        },
    )
    if distance_sensor_settings:
        world_text = update_named_block_fields(world_text, "DistanceSensor", distance_sensor_settings, apply_to_all=True)

    trace_environment_file = os.path.join(
        destination_folder,
        f"{os.path.splitext(os.path.basename(environment_file))[0]}__runtime_noise.wbt",
    )
    with open(trace_environment_file, "w", encoding="utf-8") as temp_world_file:
        temp_world_file.write(world_text)

    with open(environment_file, "w", encoding="utf-8") as runtime_world_file:
        runtime_world_file.write(world_text)

    return environment_file, original_world_text


def normalize_noise_fields(raw_settings, alias_map):
    """
    Normaliza aliases de nomes de campos vindos do JSON.
    """
    if not raw_settings:
        return None
    if not isinstance(raw_settings, dict):
        raise ValueError("Each entry inside 'sensor_noise' must be an object.")

    normalized = {}
    for key, value in raw_settings.items():
        if key.startswith("_"):
            continue
        canonical_key = alias_map.get(key, key)
        normalized[canonical_key] = value
    return normalized


def normalize_compass_lookup_table(lookup_table):
    """
    Normaliza tabelas do Compass para preservar a identidade do sensor.

    O controlador usa wb_compass_get_values() diretamente e espera componentes
    em torno de [-1, 1]. Uma tabela simplificada 0..1 altera a resposta do
    sensor mesmo com ruído zero. Quando detectamos esse shorthand, expandimos
    para uma identidade simétrica em [-1, 1].
    """
    if not isinstance(lookup_table, list) or not lookup_table:
        raise ValueError("Compass lookupTable must be a non-empty list of [x, y, noise] entries.")

    normalized_rows = []
    for row in lookup_table:
        if not isinstance(row, (list, tuple)) or len(row) != 3:
            raise ValueError("Each Compass lookupTable row must have exactly 3 values.")
        normalized_rows.append([float(row[0]), float(row[1]), float(row[2])])

    identity_like_rows = all(abs(row[0] - row[1]) < 1e-9 for row in normalized_rows)
    if not identity_like_rows:
        return normalized_rows

    xs = [row[0] for row in normalized_rows]
    if min(xs) < 0.0 or max(xs) > 1.0:
        return normalized_rows

    noise_at_zero = next((row[2] for row in normalized_rows if abs(row[0]) < 1e-9), normalized_rows[0][2])
    noise_at_one = next((row[2] for row in normalized_rows if abs(row[0] - 1.0) < 1e-9), normalized_rows[-1][2])

    return [
        [-1.0, -1.0, noise_at_one],
        [0.0, 0.0, noise_at_zero],
        [1.0, 1.0, noise_at_one],
    ]


def update_named_block_fields(world_text, node_name, field_updates, apply_to_all):
    """
    Atualiza campos de um bloco VRML/Webots por nome do nó.
    """
    pattern = re.compile(
        rf"(?P<indent>^[ \t]*){re.escape(node_name)} \{{\n(?P<body>.*?)(?P=indent)\}}",
        re.MULTILINE | re.DOTALL,
    )

    matches = list(pattern.finditer(world_text))
    if not matches:
        print(f"Warning: node '{node_name}' was not found while applying sensor noise.")
        return world_text

    def replace_match(match):
        indent = match.group("indent")
        body = match.group("body")
        updated_body = update_block_body(body, field_updates, indent + "  ")
        return f"{indent}{node_name} {{\n{updated_body}{indent}}}"

    if apply_to_all:
        return pattern.sub(replace_match, world_text)
    return pattern.sub(replace_match, world_text, count=1)


def update_block_body(body, field_updates, field_indent):
    lines = body.splitlines()
    rendered_fields = {
        field_name: render_wbt_field(field_name, value, field_indent)
        for field_name, value in field_updates.items()
    }

    updated_lines = []
    replaced_fields = set()
    index = 0

    while index < len(lines):
        stripped_line = lines[index].strip()
        field_name = next(
            (
                candidate
                for candidate in rendered_fields
                if stripped_line == candidate
                or stripped_line.startswith(candidate + " ")
                or stripped_line.startswith(candidate + "[")
            ),
            None,
        )

        if field_name is None:
            updated_lines.append(lines[index])
            index += 1
            continue

        updated_lines.extend(rendered_fields[field_name])
        replaced_fields.add(field_name)

        if stripped_line.startswith(field_name) and stripped_line.endswith("["):
            index += 1
            while index < len(lines) and lines[index].strip() != "]":
                index += 1
            if index < len(lines):
                index += 1
            continue

        index += 1

    for field_name, rendered_lines in rendered_fields.items():
        if field_name not in replaced_fields:
            if updated_lines and updated_lines[-1].strip():
                updated_lines.append("")
            updated_lines.extend(rendered_lines)

    if updated_lines:
        return "\n".join(updated_lines) + "\n"
    return ""


def render_wbt_field(field_name, value, field_indent):
    if field_name == "lookupTable":
        if not isinstance(value, list) or not value:
            raise ValueError("lookupTable must be a non-empty list of [x, y, noise] entries.")

        rendered = [f"{field_indent}{field_name} ["]
        for row in value:
            if not isinstance(row, (list, tuple)) or len(row) != 3:
                raise ValueError("Each lookupTable row must have exactly 3 values.")
            rendered.append(
                f"{field_indent}  "
                f"{format_wbt_value(row[0])} {format_wbt_value(row[1])} {format_wbt_value(row[2])}"
            )
        rendered.append(f"{field_indent}]")
        return rendered

    return [f"{field_indent}{field_name} {format_wbt_value(value)}"]


def format_wbt_value(value):
    if isinstance(value, bool):
        return "TRUE" if value else "FALSE"

    if isinstance(value, (int, float)):
        return f"{value:.15g}"

    if isinstance(value, str):
        stripped_value = value.strip()
        if stripped_value.upper() in {"TRUE", "FALSE"}:
            return stripped_value.upper()
        try:
            return f"{int(stripped_value)}"
        except ValueError:
            try:
                return f"{float(stripped_value):.15g}"
            except ValueError:
                escaped_value = stripped_value.replace('"', '\\"')
                return f'"{escaped_value}"'

    raise ValueError(f"Unsupported value type for WBT serialization: {type(value)!r}")


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
    simulation_setup_file = os.environ.get("SIMULATION_SETUP_FILE", "simulationSetup.json")

    with open(simulation_setup_file, 'r') as arquivo:
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
        sensor_noise = parse_optional_noise_config(actual_setup)
        noise_name_suffix = parse_optional_noise_label(actual_setup, sensor_noise)

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
                "-S" + str(social_adjustment_weight) +
                noise_name_suffix + "/"
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
                "-C" + str(crossover_fraction) +
                noise_name_suffix + "/"
            )

        elif metaheuristic == "WOA":
            population_size = int(actual_setup["population_size"])
            max_generations = int(actual_setup["max_generations"])
            spiral_coefficient = float(actual_setup["spiral_coefficient"])

            simulation_name = (
                os.path.splitext(os.path.basename(environment_file))[0] + "-" +
                metaheuristic + "-P" + str(population_size) +
                "-G" + str(max_generations) +
                "-SC" + str(spiral_coefficient) +
                noise_name_suffix + "/"
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
                "-NR" + str(neighborhood_radius) +
                noise_name_suffix + "/"
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
                "-CR" + str(CR) +
                noise_name_suffix + "/"
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
            runtime_environment_file = environment_file
            original_environment_contents = None
            diagnostics_enabled = os.environ.get("SIMULATION_DIAGNOSTIC_LOGS", "").strip().lower() in {"1", "true", "yes", "on"}

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

                runtime_environment_file, original_environment_contents = prepare_environment_with_noise(
                    environment_file,
                    destination_folder,
                    sensor_noise,
                )

                webots_command = ["xvfb-run", "-a", "webots", "--batch", "--no-rendering" ,  "--minimize", "--mode=fast" , runtime_environment_file] #, , , "--no-rendering" ,  "--minimize"
                webots_stdout_path = os.path.join(destination_folder, "webots_stdout.log") if diagnostics_enabled else None
                webots_stderr_path = os.path.join(destination_folder, "webots_stderr.log") if diagnostics_enabled else None
                processo_webots = start_process(
                    webots_command,
                    env=env,
                    stdout_path=webots_stdout_path,
                    stderr_path=webots_stderr_path,
                )

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

                control_stdout_path = os.path.join(destination_folder, "control_program_stdout.log") if diagnostics_enabled else None
                control_stderr_path = os.path.join(destination_folder, "control_program_stderr.log") if diagnostics_enabled else None
                control_program_process = start_process(
                    control_command,
                    stdout_path=control_stdout_path,
                    stderr_path=control_stderr_path,
                )

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
                restore_file_contents(environment_file, original_environment_contents)

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

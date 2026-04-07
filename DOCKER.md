# Uso Com Docker

Este projeto pode rodar em container. A imagem instala Webots R2025a, Python 3, `xvfb`, `ffmpeg` e as ferramentas de compilação usadas pelo controlador C do Webots.

## Estrutura Recomendada

Os resultados devem ficar fora do container, montados como volume:

```text
./simulation_results  ->  /home/simulator/IC-IA-webots_loop/simulation_results
```

Essa é a melhor opção porque o container pode ser removido ou recriado sem perder os arquivos de saída das simulações.

## Build

No diretório raiz do projeto:

```bash
docker compose build
```

O build precisa de acesso à internet para baixar o Webots R2025a a partir do repositório Debian da Cyberbotics. Durante o build, a imagem também compila o controlador C `controllers/my_controller_Micael`, então a simulação já inicia com o controlador correto dentro do container.

Ou, sem Compose:

```bash
docker build -t ic-ia-webots-loop:latest .
```

## Executar

Com Compose:

```bash
mkdir -p simulation_results
docker compose run --rm simulator
```

Sem Compose:

```bash
mkdir -p simulation_results
docker run --rm \
  --shm-size=2g \
  -v "$PWD/simulation_results:/home/simulator/IC-IA-webots_loop/simulation_results" \
  -v "$PWD/simulationSetup.json:/home/simulator/IC-IA-webots_loop/simulationSetup.json:ro" \
  ic-ia-webots-loop:latest
```

## Configuração

O arquivo `simulationSetup.json` já usa caminhos compatíveis com o container:

```json
"metaheuristic_runner_path": "/home/simulator/IC-IA-webots_loop/controllers/my_controller_Micael/python_runtime/control_program.py",
"path_to_save_simulation_files": "/home/simulator/IC-IA-webots_loop/simulation_results/"
```

Se você criar outro arquivo de setup, mantenha esses caminhos internos do container.

## Verificações Úteis

Conferir a versão do Webots dentro do container:

```bash
docker compose run --rm simulator xvfb-run -a webots --version
```

Abrir um shell no container:

```bash
docker compose run --rm simulator bash
```

## Observações

- O Webots roda em modo headless via `xvfb-run`, chamado pelo próprio `Simulation_loop.py`.
- O comando usado pelo launcher é equivalente a:
  ```bash
  xvfb-run -a webots --batch --no-rendering --minimize --mode=fast <arquivo.wbt>
  ```
- Por isso a imagem instala `xvfb`, `xauth` e `libxcb-cursor0`, que são necessários para o Webots/Qt funcionar sem interface gráfica real.
- A imagem inclui `ffmpeg`, usado para converter os gráficos gerados em JPEG.
- Os arquivos criados em `simulation_results/` ficam no host e não são apagados junto com o container.

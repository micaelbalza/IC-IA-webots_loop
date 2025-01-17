# 🛠️ Projeto de Navegação Robótica com Simulações no Webots

Este repositório contém a plataforma de simulação desenvolvida para testar o algoritmo de navegação **MHRTSN** (MetaHeuristic Real-Time Safe Navigation). A plataforma integra o **MATLAB** e o **Webots**, utilizando um fluxo automatizado para configurar, executar e salvar os resultados das simulações.

---

## 📋 **Pré-requisitos**

Antes de começar, certifique-se de ter os seguintes softwares instalados:
- **MATLAB**: Versão 2022a ou superior recomendada.
- **Webots**: Versão compatível com seu sistema operacional.

---

## ⚙️ **Configuração Inicial**

1. **Clonar o Repositório**  
   Faça o clone deste repositório para o seu computador:
   ```bash
   git clone git@github.com:micaelbalza/IC-IA-webots_loop.git
   cd seu-repositorio
   ```

2. **Definir o Padrão da Simulação**  
   Configure o arquivo `simulationSetup.json` de acordo com os parâmetros desejados.  
   Exemplo de estrutura do JSON:
   ```json
   {
    "metaheuristic": "GA",
        "population_size": "30",
        "max_generations": "10",
        "elite_count": "1",
        "crossover_fraction": "0.70",
        
        "robot_radius": "0.15",
        "alpha": "1.0",
        
        "environment": "/home/IC-IA-webots_loop/worlds/Comparisons - article 2.wbt",
        "matlab_file": "/home/IC-IA-webots_loop/controllers/my_controller_Micael/Matlab/",
        "path_to_save_simulation_files": "/home/IC-IA-webots_loop/controllers/my_controller_Micael/Matlab/swap_webots_matlab/",
        "end_point_x": "8.5",
        "end_point_y": "5.6",
        
        "simulation_repeat": "25",
        "average_time_in_each_simulation": "160",
        "webots_initiation_time": "15",
        "matlab_initiation_time": "20"
}
   ```

---

## ▶️ **Como Executar**

1. Inicie o script principal para executar as simulações:
   ```bash
   python Simulation_loop.py
   ```

2. O programa irá:
   - Ler as configurações do arquivo `simulationSetup.json`.
   - Configurar e iniciar as simulações no **Webots**.
   - Salvar os resultados em um diretório específico.

---

## 📁 **Resultados**

Os resultados das simulações serão salvos em um diretório configurado no JSON, contendo:
- **Relatórios de Desempenho**: Métricas como tempo de navegação, distância percorrida e numero de paradas...  
- **Dados Brutos**: Logs detalhados de cada simulação.  
- **Gráficos**: Visualizações geradas com base nos resultados.

---

## 🔍 **Sobre o Algoritmo MHRTSN**

O algoritmo **MHRTSN** é uma abordagem inovadora baseada em meta-heurísticas para planejamento de navegação em ambientes desconhecidos. Ele foi testado extensivamente utilizando esta plataforma, demonstrando sua eficiência em situações complexas.

---


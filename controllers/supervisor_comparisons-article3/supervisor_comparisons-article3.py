from controller import Supervisor

# Cria uma instância do supervisor
supervisor = Supervisor()

# Define o tempo de passo da simulação
timestep = int(supervisor.getBasicTimeStep())

# Define o tempo entre os movimentos (em segundos)
time_between_moves = 3  # Tempo de espera entre cada movimento

# Lista de posições pelas quais a caixa vai iterar
box_positions = [
    [-1, -3, 0.1],   # Posição inicial
    [-1.5, -2.5, 0.1],  # Segunda posição
    [-2, -2, 0.1]      # Terceira posição
]

# Inicializa variáveis para controle de tempo e estado da caixa
box_added = False
current_position_index = 0
last_move_time = 0

# Referência para a caixa
box_node = None

# Loop principal
while supervisor.step(timestep) != -1:
    # Obtém o tempo atual da simulação
    current_time = supervisor.getTime()

    # Adiciona a caixa na posição inicial se ainda não foi adicionada
    if not box_added:
        # Obtém a raiz da cena
        root = supervisor.getRoot()
        children = root.getField('children')

        # Define os parâmetros da caixa com propriedades físicas (sem DEF)
        box_string = (
            'Solid {'
            '  translation -1 -3 0.1'  # Posição inicial
            '  children ['
            '    Shape {'
            '      appearance Appearance {'
            '        material Material {'
            '          diffuseColor 1 0 0'
            '        }'
            '      }'
            '      geometry Box {'
            '        size 0.5 0.5 0.5'  # Tamanho da caixa
            '      }'
            '    }'
            '  ]'
            '  boundingObject Box {'
            '    size 0.5 0.5 0.5'
            '  }'
            '  physics Physics {'
            '    mass 1.0'
            '  }'
            '}'
        )

        # Adiciona a caixa à simulação
        children.importMFNodeFromString(-1, box_string)

        # Espera um tempo para garantir que o Webots crie o nó corretamente
        supervisor.step(timestep)

        # Obtém uma referência ao último nó adicionado (a caixa) no campo 'children'
        box_node = children.getMFNode(children.getCount() - 1)
        
        # Verifica se a caixa foi criada corretamente
        if box_node is not None:
            print("Caixa adicionada com sucesso na posição inicial.")
        else:
            print("Erro ao adicionar a caixa: não foi possível encontrar o nó da caixa adicionada.")
        
        # Define a flag para indicar que a caixa foi adicionada
        box_added = True
        
        # Marca o tempo inicial
        last_move_time = current_time

    # Realiza a movimentação iterativa entre os pontos se a caixa foi adicionada
    if box_added and box_node is not None:
        # Verifica se é hora de mover a caixa
        if current_time - last_move_time >= time_between_moves:
            # Atualiza o índice da posição atual
            current_position_index = (current_position_index + 1) % len(box_positions)
            
            # Move a caixa para a nova posição
            box_translation_field = box_node.getField('translation')
            if box_translation_field is not None:
                box_translation_field.setSFVec3f(box_positions[current_position_index])
                print(f"Caixa movida para {box_positions[current_position_index]}")
            else:
                print("Erro: Não foi possível acessar o campo 'translation' da caixa.")
            
            # Atualiza o tempo do último movimento
            last_move_time = current_time

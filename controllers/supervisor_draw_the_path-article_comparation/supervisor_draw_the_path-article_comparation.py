from controller import Supervisor

# Inicialização do supervisor
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Definindo os novos percursos fornecidos pelo usuário
dpna_ga_points = [
    [-7.5, 7.5], [-4.5, 6.7], [-3.2, 5.2], [-3, 3.2], [-3.6, 0.7], [-3.5, -2.8],
    [-2.5, -3.6], [0, -5.8], [2.5, -6.9], [6.4, -7], [7.4, -7.4]
]
mhrtsn_ga_points_new = [
    [-7.49608300, 7.50493100], [-6.53169100, 7.12914000], [-5.57204700, 7.15310700], 
    [-4.63177800, 7.16762900], [-3.77619100, 7.05412300], [-3.03350700, 6.07161100], 
    [-2.96194400, 5.10636600], [-2.98842200, 4.14661500], [-3.14068200, 3.17972300], 
    [-2.83118200, 2.90694600], [-1.85655300, 2.55288000], [-0.88578000, 2.48911100], 
    [0.07226200, 1.52488200], [0.04196000, 0.57657000], [0.25010100, -0.38047200], 
    [0.05035000, -1.31379900], [0.05738300, -2.27999300], [0.00089500, -3.18720900], 
    [-0.04311700, -4.13372200], [-0.00073300, -5.02938200], [0.01254600, -5.99538100], 
    [0.96891900, -6.91554400], [1.94944000, -7.76762200], [2.85798300, -8.25378300], 
    [3.82390400, -8.28400700], [4.78253000, -8.15945600], [5.73733800, -7.91542000], 
    [6.68389200, -7.71986600], [7.47527500, -7.54408667]
]
mhrtsn_pso_points_new = [
    [-7.49608300, 7.50493100], [-6.52851000, 7.15657100], [-5.56208800, 7.13751900], 
    [-4.59566100, 7.13296600], [-3.62867000, 6.97436100], [-3.02840000, 5.98932800], 
    [-3.01500000, 5.02287900], [-2.99660400, 4.05687000], [-3.07813200, 3.08716400], 
    [-2.09427600, 2.55968700], [-1.12254400, 2.51758100], [-0.15076800, 1.53098100], 
    [-0.01712300, 0.56002300], [0.06875400, -0.40922500], [-0.00631600, -1.37333000], 
    [0.03609000, -2.33873000], [-0.04980100, -3.30825300], [-0.02503700, -4.27440700], 
    [0.00959300, -5.24010700], [0.07138300, -6.20446400], [1.06087100, -7.17060000], 
    [2.04790100, -8.13838500], [3.01831000, -8.30942200], [3.98631900, -8.20671800], 
    [4.95144400, -7.92653700], [5.91770900, -7.93986000], [6.88338400, -7.71268200], 
    [7.54856481, -7.53473465]
]

# Funções para desenhar círculos e linhas
def draw_circle(position, color, radius=0.2):
    root = supervisor.getRoot()
    children_field = root.getField("children")
    height = 0.05
    circle_string = f"""
    DEF CIRCLE Transform {{
        translation {position[0]} {position[1]} {height}
        children [
            Shape {{
                appearance Appearance {{
                    material Material {{
                        diffuseColor {color[0]} {color[1]} {color[2]}  # Cor definida
                        emissiveColor {color[0]} {color[1]} {color[2]}  # Brilho
                    }}
                }}
                geometry Cylinder {{
                    radius {radius}
                    height 0.02
                }}
            }}
        ]
    }}
    """
    children_field.importMFNodeFromString(-1, circle_string)

def draw_line(points, color, thickness=0.1):
    root = supervisor.getRoot()
    children_field = root.getField("children")
    height = 0.05
    for i in range(len(points) - 1):
        start_pos = points[i]
        end_pos = points[i + 1]
        line_string = f"""
        DEF LINE Shape {{
            appearance Appearance {{
                material Material {{
                    diffuseColor {color[0]} {color[1]} {color[2]}  # Cor da linha
                    emissiveColor {color[0]} {color[1]} {color[2]}  # Brilho
                }}
            }}
            geometry IndexedLineSet {{
                coord Coordinate {{
                    point [
                        {start_pos[0]} {start_pos[1]} {height},  # Ponto inicial
                        {end_pos[0]} {end_pos[1]} {height}       # Ponto final
                    ]
                }}
                coordIndex [0, 1]  # Conectar os pontos
            }}
        }}
        """
        children_field.importMFNodeFromString(-1, line_string)

# Cores
cyan = (0, 1, 1)
blue = (0, 0, 1)
yellow = (1, 1, 0)

# Desenhando os percursos
# DPNA-GA (ciano)
for point in dpna_ga_points:
    draw_circle(point, cyan)
draw_line(dpna_ga_points, cyan, thickness=0.1)

# MHRTSN-GA (azul) - novo conjunto de pontos
for point in mhrtsn_ga_points_new:
    draw_circle(point, blue)
draw_line(mhrtsn_ga_points_new, blue, thickness=0.1)

# MHRTSN-PSO (amarelo) - novo conjunto de pontos
for point in mhrtsn_pso_points_new:
    draw_circle(point, yellow)
draw_line(mhrtsn_pso_points_new, yellow, thickness=0.1)

# Loop principal para continuar a simulação
while supervisor.step(timestep) != -1:
    pass

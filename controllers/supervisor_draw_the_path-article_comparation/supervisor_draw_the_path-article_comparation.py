from controller import Supervisor

# Inicialização do supervisor
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Percursos fornecidos
rpso_points = [
    [-9.5, -6.5], [-8.5, -7.5], [-7.5, -6.5], [-6.5, -6.5], [-5.5, -6.5], [-4.5, -7.5],
    [-3.5, -6.5], [-2.5, -5.5], [-1.5, -4.5], [-0.5, -3.5], [0.5, -3.5], [1.5, -3.5],
    [2.5, -2.5], [3.5, -2.5], [4.5, -1.5], [5.5, -0.5], [6.5, 0.5], [7.5, 1.5],
    [8.5, 2.5], [8.5, 3.5], [9.5, 4.5], [8.5, 5.5]
]
apfrpso_points = [
    [-9.5, -6.5], [-8.5, -6.5], [-7.5, -7.5], [-6.5, -6.5], [-5.5, -5.5], [-4.5, -5.5],
    [-3.5, -5.5], [-2.5, -4.5], [-1.5, -3.5], [-0.5, -2.5], [0.5, -1.5], [1.5, -0.5],
    [2.5, -0.5], [3.5, 0.5], [4.5, 0.5], [5.5, 1.5], [5.5, 2.5], [6.5, 3.5], [7.5, 4.5],
    [8.5, 5.5]
]
mhrtsn_ga_points = [
    [-9.49607900, -6.49506100],
    [-8.52797700, -5.50908600],
    [-7.55822100, -4.52270800],
    [-6.59163700, -4.35910200],
    [-5.62660700, -4.31902300],
    [-4.66134600, -4.27733000],
    [-3.70637100, -3.57468600],
    [-3.43304700, -2.60034500],
    [-2.80914100, -1.63250500],
    [-2.06416000, -0.64756800],
    [-1.85676300,  0.32320500],
    [-1.05293800,  1.29586600],
    [-0.90154300,  2.26342300],
    [ 0.08267400,  3.23509300],
    [ 1.05807500,  3.95132200],
    [ 2.02505000,  3.96458200],
    [ 2.99335800,  4.05824500],
    [ 3.95831000,  4.16613700],
    [ 4.92437500,  4.63939700],
    [ 5.89126100,  5.35450900],
    [ 6.86248200,  5.91382600],
    [ 7.82806800,  5.94237000],
    [ 8.49720744,  5.64303166]
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
green = (0, 1, 0)
cyan = (0, 1, 1)
blue = (0, 0, 1)
yellow = (1, 1, 0)

# Desenhando os percursos
# rPSO (verde)
for point in rpso_points:
    draw_circle(point, green)
draw_line(rpso_points, green, thickness=0.1)

# apfrPSO (ciano)
for point in apfrpso_points:
    draw_circle(point, cyan)
draw_line(apfrpso_points, cyan, thickness=0.1)

# MHRTSN-GA (azul)
for point in mhrtsn_ga_points:
    draw_circle(point, blue)
draw_line(mhrtsn_ga_points, blue, thickness=0.1)


# Loop principal
while supervisor.step(timestep) != -1:
    pass

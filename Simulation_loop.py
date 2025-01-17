import subprocess
import os
import time
import json

# O problema que vc precisa resolver agora é a chamada certa do matlab, passando as variaveis pela chamada tbm. oque no momento nao está ocorrendo. precisa ajustar as funcloes no matlab GA_controlProgram e PSO_comtrol_program  e nesse arquivo colocar para enviar as variaveis via chamada.


def criar_pasta_principal(nome_pasta_principal):
    caminho_pasta_principal = os.path.join(path_to_save_simulation_files, nome_pasta_principal)
    try:
        os.makedirs(caminho_pasta_principal)
    except FileExistsError:
        print(f"The folder {caminho_pasta_principal} already exists.")
        print("\n Nothing will be done! \n")
        return False

def clean_folder(caminho_pasta):
    for arquivo in os.listdir(caminho_pasta):
        arquivo_path = os.path.join(caminho_pasta, arquivo)
        try:
            if os.path.isfile(arquivo_path):
                os.unlink(arquivo_path)
        except Exception as e:
            print(f"Erro ao excluir {arquivo_path}: {e}")




if __name__ == "__main__":
    
    # Open the Json file
    with open('simulationSetup.json', 'r') as arquivo:
        simulation_setup = json.load(arquivo)
    

    for index, actual_setup in enumerate(simulation_setup, start=1): # Type of simulations  
        # reading the parameters of the current simulation
        metaheuristic = actual_setup["metaheuristic"]
        
        robot_radius = float(actual_setup["robot_radius"])
        alpha = float(actual_setup["alpha"])
        
        environment_file = actual_setup["environment"] # file that describes the environment
        path_to_save_simulation_files = actual_setup["path_to_save_simulation_files"] 

        simulation_repeat = int(actual_setup["simulation_repeat"])
        simulation_time = float(actual_setup["average_time_in_each_simulation"])
        Webots_initiation_time = float(actual_setup["webots_initiation_time"])
        Matlab_initiation_time = float(actual_setup["matlab_initiation_time"])
        end_point_x = float(actual_setup["end_point_x"])
        end_point_y = float(actual_setup["end_point_y"])

        if metaheuristic == 'PSO':
            population_size = int(actual_setup["population_size"])
            max_generations = int(actual_setup["max_generations"])
            self_adjustment_weight = float(actual_setup["self_adjustment_weight"])
            social_adjustment_weight = float(actual_setup["social_adjustment_weight"])
            Matlab_file = actual_setup["matlab_file"]
            matlab_parameters = [population_size, max_generations, self_adjustment_weight, social_adjustment_weight, robot_radius, alpha]
            matlab_parameters_str = "','".join(map(str, matlab_parameters)) 
            #matlab_metaheuristic_function = "PSO_ControlProgram"
            simulation_name = os.path.splitext(os.path.basename(environment_file))[0] + "-" + metaheuristic + "-P"+ str(population_size) + "-G" + str(max_generations) + "-C" + str(self_adjustment_weight) + "-S" + str(social_adjustment_weight) + "/"

        elif metaheuristic == "GA":
            population_size = int(actual_setup["population_size"])
            max_generations = int(actual_setup["max_generations"])
            elite_count = int(actual_setup["elite_count"])
            crossover_fraction = float(actual_setup["crossover_fraction"])
            Matlab_file = actual_setup["matlab_file"]
            matlab_parameters = [population_size, max_generations, elite_count, crossover_fraction, robot_radius, alpha]
            matlab_parameters_str = "','".join(map(str, matlab_parameters))
            #matlab_metaheuristic_function = "GA_ControlProgram"
            simulation_name = os.path.splitext(os.path.basename(environment_file))[0] + "-" + metaheuristic + "-P"+ str(population_size) + "-G" + str(max_generations) + "-E" + str(elite_count) + "-C" + str(crossover_fraction) + "/"
        else:
            print("Error - Reading not implemented - metaheuristic fail")

        erro = criar_pasta_principal(simulation_name)

        if erro != False:
            
            time_limit_count = 0
            process_termination_count = 0
            simulations_time_limit = []
            simulations_process_termination = []

            for current_simulation_number in range(1, simulation_repeat + 1): # repetition of simulations
                
                current_simulation_number_str = str(current_simulation_number).zfill(2)  # Completing with '0' on the left. '1' --> '01'

                destination_folder = path_to_save_simulation_files + simulation_name + current_simulation_number_str + "/"

                os.makedirs(destination_folder, exist_ok=True)

                clean_folder(destination_folder)

                os.makedirs(destination_folder + "request/", exist_ok=True) # create request folder
                os.makedirs(destination_folder + "response/", exist_ok=True) # create response folder


                # Init Webots
                os.environ["DESTINATION_FOLDER"] = destination_folder
                end_point_str = f"{end_point_x},{end_point_y}"
                os.environ["END_POINT"] = end_point_str
                time.sleep(1)
                webots_command = ["webots", "--no-rendering", "--batch", "--minimize", "--mode=fast" , environment_file] #, 
                processo_webots = subprocess.Popen(webots_command)
                time.sleep(Webots_initiation_time) # waiting webots start up
                print("Webots has been started.")
                
                # Iniciar o Matlab
                work_path = path_to_save_simulation_files + simulation_name + current_simulation_number_str + "/"
                if metaheuristic == 'PSO':
                    matlab_command = ["matlab", "-sd", Matlab_file, "-batch", f"PSO_ControlProgram('{work_path}','{matlab_parameters_str}')"]
                elif metaheuristic == "GA":
                    matlab_command = ["matlab", "-sd", Matlab_file, "-batch", f"GA_ControlProgram('{work_path}','{matlab_parameters_str}')"]

                processo_matlab = subprocess.Popen(matlab_command)
                print("work_path")
                print(work_path)

                time.sleep(Matlab_initiation_time)  # waiting matlab start up
                print("Matlab has been started.")

                time_collected_from_the_start_of_the_simulation = time.time() # Collect intial time

                time_passed = False 
                matlab_ended = False

                while not matlab_ended and not time_passed:  # checks if time has passed, or if the matlab process has ended (which means that the robot has reached the objective point)
                    actual_time = time.time()
                    if processo_matlab.poll() is not None:
                        matlab_ended = True
                        process_termination_count += 1
                        simulations_process_termination.append(current_simulation_number_str)

                    if actual_time - time_collected_from_the_start_of_the_simulation > simulation_time:
                        time_passed = True
                        time_limit_count += 1 # Log count
                        simulations_time_limit.append(current_simulation_number_str) # saving simulation name --> time_limit
                        print("Timeout has been reached")

                
                # The simulation time has ended or the robot has reached the objective point        
                subprocess.Popen(["pkill", "matlab"])
                subprocess.Popen(["pkill", "webots"])
                subprocess.run(['pkill', '-f', '/home/micaelbalza/Desktop/IC-IA-webots_loop/controllers/my_controller_Micael/my_controller_Micael'])
                print(" \n The simulation " +  simulation_name  + current_simulation_number_str + " finished \n " )
                time.sleep(130) # Waiting to reestablish connection 
        
        # Recording count information in a log file
        log_filename = os.path.join(path_to_save_simulation_files, simulation_name, "log.txt")
        with open(log_filename, "w") as log_file:
            log_file.write("Number of simulations terminated due to time limit: {}\n".format(time_limit_count))
            log_file.write("Simulations terminated due to time limit:\n")
            for sim in simulations_time_limit:
                log_file.write(sim + "\n")
            
            log_file.write("\n")
            
            log_file.write("Number of simulations terminated by MATLAB process termination: {}\n".format(process_termination_count))
            log_file.write("Simulations terminated by MATLAB process termination:\n")
            for sim in simulations_process_termination:
                log_file.write(sim + "\n")
    print("All simulations are over - the program has ended")
    

        



            


   













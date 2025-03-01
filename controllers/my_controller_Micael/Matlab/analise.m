



num_de_deslocamentos = 23; %total de deslocamentos
x1 = 20; % Deslocamento inicial plotado no grafico
x2 = 23; % deslocamento final 

cont_m_displacement = 0; %displacement control assistant(for read and save txt file)
pR = [];
Z = [];
pO_vector = [];


while (cont_m_displacement <= num_de_deslocamentos)
    
    [K_points,m_displacement, pO_size, pR, final_objective, pDP, pO] = automatic_reading(cont_m_displacement, pR);
    
    myFolder = '/home/micaelbalza/Desktop/IC-IA-webots/controllers/my_controller_Micael/Matlab/swap_webots_matlab/response';
    
    % Check to make sure that folder actually exists.  Warn user if it doesn't.
    if ~isfolder(myFolder)
        erro_Message = sprintf('Error: The following folder does not exist:\n%s\nPlease specify a new folder.', myFolder);
        uiwait(warndlg(erro_Message));
        myFolder = uigetdir(); % Ask for a new one.
        if myFolder == 0
             % User clicked Cancel
             return;
        end
    end
    
    
    
    % Get a list of all files in the folder with the desired file name pattern.
    filePattern = fullfile(myFolder, '*.txt'); % search txt files
    
    aux = 0;
    
    while aux == 0
        theFiles = dir(filePattern);
        for k = 1 : length(theFiles)
            baseFileName = theFiles(k).name;
            fullFileName = fullfile(theFiles(k).folder, baseFileName);
            searched_file = append(string(cont_m_displacement),".txt");
            %disp(fullFileName)
            pause(0.1);
            if baseFileName == searched_file % if searched file was found()
                aux = 1;
                break
            end
        end
    end

    file = fopen(fullFileName, "r");
    ignore_line = fgetl(file);
    string_z = regexp(fgetl(file), " ", 'split'); % read k_points
    z = [str2double(string_z(1)),str2double(string_z(2))];

    % Qual eu quero imprimir
    if (cont_m_displacement >= x1== true) & (cont_m_displacement <= x2 == true)
        
        for c = 1:pO_size
                    plot(pO(c,1),pO(c,2),"-o")
                    hold on
                    %pause(0.1)
        end
                
        
        %hold on
        %viscircles(pR(m_displacement),1)
        hold on
        plot(final_objective(1),final_objective(2),'diamond')
        hold on 
        plot(z(1),z(2),"*")
        hold on
        plot(pR(m_displacement+1,1),pR(m_displacement+1,2),"square")
       
        %function stop = 
(optimValues,state)
        %swarm_PSO = optimValues.iteration;
        
        %hold on 
        %plot(population(:,1),population(:,2),'x')%plotar os indivisuos de cada geracao -- GA
        %plot em 3d
        %hold on 
        %fsurf(@(x,y) gJ([x,y],pDP, pO, pO_size, pR, final_objective, 1), [-10 10 -10 10]) 
        %%{
    end
    cont_m_displacement = cont_m_displacement+1;
    Z = [Z;z];
    pO_vector = [pO_vector;pO];
end

disp("Leitura finalizada")


x1 = 23;
x2 = 23;
while x1<=x2
    for c = 1:pO_size
                    plot(pO(c,1),pO(c,2),"-o")
                    hold on
                    %pause(0.1)
        end
                
        
        %hold on
        %viscircles(pR(m_displacement),1)
        hold on
        plot(final_objective(1),final_objective(2),'diamond')
        hold on 
        plot(Z(x1,1),Z(x1,2),"*")
        hold on
        plot(pR(x1+1,1),pR(x1+1,2),"square")
       
        %function stop = pswoutfun(optimValues,state)
        %swarm_PSO = optimValues.iteration;
        
        %hold on 
        %plot(population(:,1),population(:,2),'x')%plotar os indivisuos de cada geracao -- GA
        %plot em 3d
        %hold on 
        %fsurf(@(x,y) gJ([x,y],pDP, pO, pO_size, pR, final_objective, 1), [-10 10 -10 10]) 
        %%{
        route = [pR; z];
        plot(route(:,1), route(:,2), '-k');
    x1=x1+1;
end

%PLOT quadrado obstáculo
q1=[
    1.42 3.61;
    2.42 3.61;
    2.42 2.61;
    1.42 2.61
    1.42 3.61];

q2=[
    3 2.5;
    5 2.5;
    5 1.5;
    3 1.5
    3 2.5];
hold on
plot(q1(:,1),q1(:,2), '-b');
hold on
plot(q2(:,1),q2(:,2), '-b');







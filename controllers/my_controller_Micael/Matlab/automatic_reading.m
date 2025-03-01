function [K_points,m_displacement, pO_size, pR, final_objective, pDP, pO] = automatic_reading(cont_m_displacement, pR, destination_path)
    %format long ----- configura 15 digitos
    %format short----- 4 digitos


    myFolder = fullfile(destination_path, 'request');

    
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
    
    %cont_m_displacement = 0;
    aux = 0;
    
    while aux == 0
        theFiles = dir(filePattern);
        for k = 1 : length(theFiles)
            baseFileName = theFiles(k).name;
            fullFileName = fullfile(theFiles(k).folder, baseFileName);
            %searched_file = append(string(cont_m_displacement),".txt");
            searched_file = string(cont_m_displacement) + ".txt";
            %disp(fullFileName)
            pause(0.1);
            if baseFileName == searched_file % if searched file was found()
                aux = 1;
                break
            end
        end
    end
    
    
    %file was found - open and interpret the file
    file = fopen(fullFileName, "r");
    
    string_K_points = regexp(fgetl(file), "/-/", 'split'); % read k_points
    K_points = str2double(string_K_points(2));
    
    string_m_displacement = regexp(fgetl(file), "/-/", 'split');%read m_displacement
    m_displacement = str2double(string_m_displacement(2));
    
    string_pO_size = regexp(fgetl(file), "/-/", 'split');%read pO.size
    pO_size = str2double(string_pO_size(2));
    
    ignore_line = fgetl(file);
    
    ignore_line = fgetl(file);
    
    %pR
    if cont_m_displacement == 0
        pR = [];
    end
    string_pR = regexp(fgetl(file), "/-/", 'split');
    pR = [pR; str2double(string_pR(1)) str2double(string_pR(2))];
    
    ignore_line = fgetl(file);
    ignore_line = fgetl(file);
    
    %final_objective
    final_objective = [];
    string_final_objective = regexp(fgetl(file), "/-/", 'split');
    final_objective = [str2double(string_final_objective(1)) str2double(string_final_objective(2))];
    
    ignore_line = fgetl(file);
    ignore_line = fgetl(file);
    
    %pDP
    pDP = [];
    for k = 1 : K_points
        string_pDP = regexp(fgetl(file), "/-/", 'split');
        pDP = [pDP; str2double(string_pDP(1))  str2double(string_pDP(2))];
    
    end
    
    ignore_line = fgetl(file);
    ignore_line = fgetl(file);
    
    %pO
    pO = [];
    for k = 1 : pO_size
        string_pO = regexp(fgetl(file), "/-/", 'split');
        pO = [pO; str2double(string_pO(1))  str2double(string_pO(2))];
    end

    %CHAMAR O GA
    %que chama as funcoes de fitness
    %volta pro arquivo
end









function [cont_m_displacement] = automatic_save(z,m_displacement, destination_path)   
    
    %myFolder = '/home/micaelbalza/Desktop/IC-IA-webots/controllers/my_controller_Micael/Matlab/swap_webots_matlab/response/';
    myFolder = fullfile(destination_path, 'response/');

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

    
    output_file = append(string(m_displacement),".txt");
    output_file = append(myFolder,output_file);

    fileID = fopen(output_file,'w');
    
    fprintf(fileID,'pOL(m)(metaheuristic result): X Y\n');
    fprintf(fileID,'%f %f',z(1),z(2));
    fclose(fileID);

    cont_m_displacement = m_displacement + 1;

    
end



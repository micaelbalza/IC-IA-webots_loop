%  Controle geral do programa, com etapas bem definidas:
%  Control_program()
%    |    |--> Automatic_reading() <--   <--   <--  <--  <--  <--
%    |    |--> Control_program(metaheuristic) <--> fitness()    /|\
%    |    |--> output                                            |
%    |    |--> Automatic_save()                                  |
%    |    |--> Wait  -->  -->  --> -->  -->  -->  -->   -->  --> |
%   \|/


function ControlProgram(destination_path)
	
	% ram()
	    % definitions and parameters GA
	    % https://www.mathworks.com/help/gads/gaoptimset.html#d124e56002
	    % https://www.mathworks.com/help/gads/genetic-algorithm-options.html#f7820
	    
	    %
	    options = gaoptimset(@ga);
	    options.PopulationSize = 20; %50
	    options.Generations = 20;
	    options.EliteCount = 2;
	    options.CrossoverFraction = 0.8;
	    %}
	    % Alguns dos 3 abaixo estaá bugando o raio do deslocamento
	    %options.SelectionFcn = 'selectionstochunif'; %stochastic uniform
	    %options.CrossoverFcn = 'crossoverscattered';
	    %options.MutationFcn = @mutationadaptfeasible; % (0,1)
	    %options.MutationFcn = 'mutationgaussian';
	    %options.MutationFcn = {@mutationadaptfeasible, 0.2, [0.1 1]};    
	    %%%options.CrossoverFraction = 0.6;
	    
	    %Opitions --> PSO
	    %{
	    options.SwarmSize = 30;
	    options.MaxIterations = 10;
	    options.SelfAdjustmentWeight = 1;   %default 1.49
	    options.SocialAdjustmentWeight = 1.5; %default 1.49
	    

	    %}

	    robotRadius = 0.15;
	    alpha = 1.4; % OBS: Alpha >= 1

	    tempos  = [];
	    tempos2 = [];% Monitoramento da diversidade genética
	options.Vectorized = 'off'; % Desabilita 
	    cont_m_displacement = 0; %displacement control assistant(for read and save txt file)
	    pR = [];
	    

	    
	    global Rd 
	    Rd= 1;   % Radius (Rd) in meters --> sub radius of action (Rd in fitness too)
	    
	    client = tcpclient("localhost",12345)% TCP connection

	    while (1) 
		[K_points,m_displacement, pO_size, pR, final_objective, pDP, pO] = automatic_reading(cont_m_displacement, pR, destination_path);
		
		dist_max = 1;
		pDP_space = C_space(pDP, robotRadius, alpha); %Modification 1 - fitness function
		
		if(m_displacement == 0)
		    [l c] = size(pR);
		    d = sqrt( ((pR(l,1) - final_objective(1,1) )^2 + ( pR(l,1)  - final_objective(1,2)) )^2 );
		    %%Final_distance - robot < 0.2 --> is it
		    if (d < .2)
		        break;
		    end
		
		end
		
		%disp('Global Objetive Found!')


		%------ control_program ----
		func_fitness = @(x)gJ(x,pDP_space, pO, pO_size, pR, final_objective, 1);
		[l c] = size(pR);
		a = pR(l,1);
		b = pR(l,2);
		
		
		
		
		tic         % pair 1: tic
		t1 = cputime;
		%GA
		[z fval exitflag output population scores]  = ga(func_fitness, 2, [],[],[],[], [(a-Rd) (b-Rd)], [(a+Rd) (b+Rd)], [], options);
		
		%PSO
		%[z,fval,exitflag,output] = particleswarm(func_fitness, 2, [(a-Rd) (b-Rd)], [(a+Rd) (b+Rd)], options);
		 
		t2 = cputime -t1;
		
		tempos2 = [toc,tempos2];  % pair 1: to
		tempos = [t2 ,tempos]; 
	    
		%------ control_program ----
		
		[cont_m_displacement] = automatic_save(z, cont_m_displacement, destination_path);
		
		while true
		    write(client,"read")%send TCP flag to read txt file
		    disp('enviando')
		    confirm = read(client,8,'char');
		    if(confirm == "received")
		        confirm = "";
		        disp('Confirmado')
		        break;
		    end
		    pause(0.5)
		end

		%pause(0.1);
		
		%%{
		
		% Creates an invisible figure
		fig = figure('Visible', 'off');

		for c = 1:pO_size
		    plot(pO(c,1),pO(c,2),"-o")
		    hold on
		    %pause(0.1)
		end
		

		%hold on
		%viscircles(pR(m_displacement),1)
		%hold on
		plot(final_objective(1),final_objective(2),'diamond')
		%hold on 
		plot(z(1),z(2),"*")
		%hold on
		plot(pR(m_displacement+1,1),pR(m_displacement+1,2),"square")
	       
		%function stop = pswoutfun(optimValues,state)
		%swarm_PSO = optimValues.iteration;
		
		%hold on 
		plot(population(:,1),population(:,2),'x')%plotar os individuos de cada geracao -- GA
		%plot em 3d
		%hold on 
		%fsurf(@(x,y) gJ([x,y],pDP, pO, pO_size, pR, final_objective, 1), [-20 20 -20 20]) 
		%hold on
		%%{
		alfa = 0;
		circunf = [];
		radio = 3;
		%{
		while(alfa <= 360)
		    circunf = [circunf; pR(1) + sind(alfa)*radio pR(2) + cosd(alfa)*radio];
		    alfa = alfa + 10;
		end
		plot(circunf(:,1),circunf(:,2),'g--');
		%}
		%plot(pDP(:,1),pDP(:,2),'-.', 'Color', 'red');
		hold on
		plot(pDP_space(:,1),pDP_space(:,2),'-.', 'Color', 'blue');
		
		centers = [pR(m_displacement+1,1),pR(m_displacement+1,2)];
		%hold on
		%viscircles(centers ,1);
		%hold on
		%%}
		%%}        [linha coluna] = size(pR); % distancia final ao
		%%objetivo.
		d = sqrt( (z(1) - final_objective(1,1) )^2 + ( z(2)  - final_objective(1,2))^2 );

		if (d < .2)
		    break;
		end


	    end



	route_distance = 0; % Initialize the total distance.
	max_distance = 0; % Initialize the highest distance

	for i = 1:m_displacement
	    aux = sqrt((pR(i+1, 1) - pR(i, 1))^2 + (pR(i+1, 2) - pR(i, 2))^2);
	    route_distance = route_distance + aux;
	    %fprintf('Distancia deslocamento %d é de %f\n', i, aux);
	    
	    % Atualiza a maior distância, se necessário
	    if aux > max_distance
		max_distance = aux;
	    end
	end

	fprintf('A maior distância percorrida é de %f\n', max_distance);





	sum_tempos = 0;
	for i = 1 : m_displacement+1
	    sum_tempos = sum_tempos + tempos(i);
	end
	sum_tempos2 = 0;
	for i = 1 : m_displacement+1
	    sum_tempos2 = sum_tempos2 + tempos2(i);
	end

	tempos;%CPU
	sum_tempos;%CPUz
	m_displacement;
	pR;
	route_distance;
	sum_tempos2;

	file = fullfile(destination_path, 'log.txt');
	fileID = fopen(file, 'w');

	% Verifica se o arquivo foi aberto corretamente
	if fileID == -1
	    error('Não foi possível abrir o arquivo.');
	end

	% Escreve os dados das variáveis no arquivo
	fprintf(fileID, 'Dados das Variáveis:\n\n');
	fprintf(fileID, 'Tempo de CPU: \n');
	fprintf(fileID, ' %12.8f \n\n',tempos);
	fprintf(fileID, 'Somatório do tempo de CPU gasto: \n');
	fprintf(fileID, ' %6.8f \n\n', sum_tempos);
	fprintf(fileID, 'Número de deslocamentos: \n');
	fprintf(fileID, ' %d \n\n', m_displacement);
	fprintf(fileID, 'Pontos do robô (centros): \n');
	for i = 1:size(pR, 1)
	    fprintf(fileID, '%6.8f %6.8f\n', pR(i, 1), pR(i, 2));
	end
	fprintf(fileID, '\n\n Distância percorrida: \n');
	fprintf(fileID, ' %6.8f \n\n', route_distance);
	fprintf(fileID, 'Somatório do tempo de relógio gasto: \n');
	fprintf(fileID, ' %6.8f \n\n', sum_tempos2);

	% Fecha o arquivo
	fclose(fileID);

	disp('Log salvo com sucesso.');

	
	file_name = 'Data_and_displacement.fig';
	full_file_path = fullfile(destination_path, file_name);
	savefig(full_file_path);

	route = [pR; z]; %tracejado
	plot(route(:,1), route(:,2), '-k'); %tracejado

	file_name = 'Route.fig';
	full_file_path = fullfile(destination_path, file_name);
	savefig(full_file_path);

	disp('figuras salvas com sucesso.');
	close
	clear
	clc

	exit; %close Matlab
end


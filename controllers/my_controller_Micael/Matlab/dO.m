function [output] = dO(pGA_m,pO_m,pO_size, individual)  % Eq. 15
    aux = 1000000000.0; % uma distancia improvavel para o espaco de trabalho
    for i = 1 : pO_size
        f_ed = sqrt( (pGA_m(individual,1) - pO_m(i,1) )^2 + ( (pGA_m(individual,2) - pO_m(i,2)) )^2  );
        if f_ed < aux
            aux = f_ed;
        end
    %output tava aqui
    end
    output = aux;
end
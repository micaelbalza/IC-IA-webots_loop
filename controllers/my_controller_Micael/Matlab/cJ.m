function [output1] = cJ(pGA_m, pR,individual) % Eq 17   ( confirmar)
    %Explicação retirada da implementação em C, por isso esses termos e variaveis
    %// Funciona assim: A circunferencia atual tem mais chance - a parte da circunferencia 
    % atual que tem intersecção com as anteriores ou são anteriores é colocado uma penalidade de 1000
    %// desta forma ele não anda pra tras - e locais já "visitados" não são reavaliados
    %//pGA_m --> Nó que contém os indivíduos desta geração analizada
    %//pR_list --> linked list que contém a posição atual do robo(ultima) e o historico de todas as posições anteriores
    
    Rd = 1;   % Radius (Rd) in meters --> sub radius of action 
    Z = 1000.0; % (Eq 17)
    %disp(pGA_m)
    visited_previously = 0;

    [l c] = size(pR);
    for i = 1 : l-1
        if sqrt( (pGA_m(individual,1) - pR(i,1) )^2 + ( (pGA_m(individual,2) - pR(i,2)) )^2  ) <= Rd 
            visited_previously = 1;
        end
    end
    %e_dist = sqrt( (pGA_m(individual,1) - pR(l,1) )^2 + ( (pGA_m(individual,2) - pR(l,2)) )^2  );
    if visited_previously == 1  %  if e_dist <= Rd && visited_previously == true
        output1 = Z;
    else
        output1 = 1.0;
    end

end

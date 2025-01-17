function [fitness] = gJ(pGA_m,pDP, pO_m, pO_size, pR, objective, individual)
    % pGA_m foi pensado inicialmente como uma struct contendo x,y de todos
    % os individuos
    % essa implementação em matlab usa como 1 individuo só
    % por isso que individual é sempre 1
    % gJ(x, pO_m, pO_size, pR, final_objective, 1); --> chamada
    
   % A = dOF(pGA_m,objective, individual); % OK
    %if (beta(pO_size) == 1)
     %   B = 1 / dO(pGA_m,pO_m,pO_size, individual) ;
      %  C = cJ(pGA_m, pR, 1) ;
       % D = aJ(pGA_m,pDP,individual);
    %else
     %   B = 0;
      %  C = 0;
      %  D = 0;
    %end

    %fitness = A+B+C+D;
%end  


A = dOF(pGA_m,objective, individual); % OK
    if (beta(pO_size) == 1) % tem obstaculo
        B = dO(pGA_m,pO_m,pO_size, individual) ;
        Bb = (1/B);
        C = cJ(pGA_m, pR, 1);
        D = aJ(pGA_m,pR,pDP,individual);
    else
        %B = 0;
        C = 0;
        Bb = 0;
        D = aJ(pGA_m,pR,pDP,individual);
    end


    fitness = A+Bb+C+D;
end  
 
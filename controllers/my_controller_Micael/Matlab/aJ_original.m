function [output2] = aJ_original(pGA_m, pR, pDP, individual_number) % Eq 18
   

    in = inpolygon(pGA_m(individual_number, 1), pGA_m(individual_number, 2), pDP(:, 1), pDP(:, 2));
    
    
    if in == true
        output2 = 0; %  antes tava 0, mas na dissertaççaod de Atila tá 1
    else
        output2 = Inf;
    end 

end 
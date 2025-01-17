function [distance] = dOF(pGA_m,objective, individual)
    distance = sqrt( (pGA_m(individual,1) - objective(1,1) )^2 + ( (pGA_m(individual,2) - objective(1,2)) )^2  );
end 
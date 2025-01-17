function [b] = beta(pO_size)  % Eq. 16
    if(pO_size > 0)
        b = 1;
    else
        b = 0;
    end
end
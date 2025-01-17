function output2 = aJ(pGA_m, pR, pDP_space, individual_number)
    identifier_of_robot_current_position = size(pR, 1);
    % Verifica se o ponto de interesse está dentro do polígono
    dentro = inpolygon(pGA_m(individual_number, 1), pGA_m(individual_number, 2), pDP_space(:, 1), pDP_space(:, 2));

    if dentro
        % Verifica se a semi-reta passa para fora do polígono em algum momento
        intersecta = false;
        for i = 1:size(pDP_space, 1)
            next_index = mod(i, size(pDP_space, 1)) + 1;
            if intersectLines(pGA_m(individual_number, :), pR(identifier_of_robot_current_position,:), pDP_space(i, :), pDP_space(next_index, :))
                intersecta = true;
                break;
            end
        end

        if intersecta
            output2 = Inf; % A semi-reta passa para fora do polígono
        else
            output2 = 0; % A semi-reta está completamente contida no polígono
        end
    else
        output2 = Inf; % O ponto de interesse está fora do polígono
    end
end



function intersecta = intersectLines(p1, p2, p3, p4)
    % Verifica se a linha definida pelos pontos p1 e p2 intersecta a linha definida pelos pontos p3 e p4
    intersecta = (orientation(p1, p3, p4) ~= orientation(p2, p3, p4)) && ...
                 (orientation(p1, p2, p3) ~= orientation(p1, p2, p4));
end



function o = orientation(p1, p2, p3)
    % Calcula a orientação de três pontos (p1, p2, p3) em sentido horário (1), anti-horário (-1) ou colinear (0)
    val = (p2(2) - p1(2)) * (p3(1) - p2(1)) - (p2(1) - p1(1)) * (p3(2) - p2(2));
    if val == 0
        o = 0;
    elseif val > 0
        o = 1; % Sentido horário
    else
        o = -1; % Sentido anti-horário
    end
end




   
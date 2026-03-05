
function [Leader_pos, Leader_score] = run_WOA(fobj, lb, ub, SearchAgents_no, Max_iter, b)

    dim = length(lb);
    X = repmat(lb, SearchAgents_no, 1) + rand(SearchAgents_no, dim) .* (repmat(ub - lb, SearchAgents_no, 1));
    fitness = arrayfun(@(i) fobj(X(i,:)), 1:SearchAgents_no)';
    [Leader_score, idx] = min(fitness);
    Leader_pos = X(idx, :);

    for t = 1:Max_iter
        a = 2 - t * (2 / Max_iter);
        for i = 1:SearchAgents_no
            r1 = rand(); r2 = rand();
            A = 2 * a * r1 - a;
            C = 2 * r2;
            l = rand() * 2 - 1;
            p = rand();

            if p < 0.5
                if abs(A) >= 1
                    rand_idx = randi(SearchAgents_no);
                    D = abs(C * X(rand_idx,:) - X(i,:));
                    X(i,:) = X(rand_idx,:) - A * D;
                else
                    D = abs(C * Leader_pos - X(i,:));
                    X(i,:) = Leader_pos - A * D;
                end
            else
                D = abs(Leader_pos - X(i,:));
                X(i,:) = D .* exp(b * l) .* cos(2 * pi * l) + Leader_pos;
            end

            % Clamping to bounds
            X(i,:) = max(min(X(i,:), ub), lb);

            f_new = fobj(X(i,:));
            if f_new < Leader_score
                Leader_score = f_new;
                Leader_pos = X(i,:);
            end
        end
    end
end
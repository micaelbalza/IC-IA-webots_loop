function [output] = C_space(pDP, robot_radius, alpha)

    % Testing alpha factor, ensuring that it will always be greater than or equal to 1.
    if (alpha < 1)
        alpha = 1;
    end
    
    % Number of points in the polygon
    numPoints = size(pDP, 1);
    
    % Initialize the new polygon
    newPolygon = [];
    

    % ---- Fist segment:
    %Fist point
    direction = pDP(1+1,:) - pDP(1,:); 
    
    % Normalize the direction vector
    direction = direction / norm(direction);
    
    % Calculating the normal vector
    normal = [-direction(2), direction(1)];
    
    % Shifting the points in both directions
    p1_new = pDP(1,:) + alpha * robot_radius * normal; 
    %p2_new = pDP(1+1,:) + alpha * normal; 
    
    newPolygon = [newPolygon; p1_new]; % We only need the first point.
    
    
    % ---- intermediate segments:
    % Calculating the shifting segments
    for i = 1:numPoints - 1
        % Direction vector between two points
        direction = pDP(i+1,:) - pDP(i,:);
        
        % Normalize the direction vector
        direction = direction / norm(direction);
        
        % Calculating the normal vector
        normal = [-direction(2), direction(1)];
        
        % Shifting the points in both directions
        p1_new = pDP(i,:) + alpha * robot_radius * normal; 
        p2_new = pDP(i+1,:) + alpha * robot_radius * normal; 
        
        % Calculating the equation of the straight line for the shifting segment
        m = (p2_new(2) - p1_new(2)) / (p2_new(1) - p1_new(1));
        b = p1_new(2) - m * p1_new(1);
        
        % Calculating the intersection between the actual segment and next segment
        p_intersection = [NaN NaN]; % Inicialize as NaN if there ir no intersection
        if i < numPoints - 1
            % Direction vector between the two points of the next segment 
            next_direction = pDP(i+2,:) - pDP(i+1,:);
            
            % Normalize the direction vector
            next_direction = next_direction / norm(next_direction);
            
            % Calculating the normal vector
            next_normal = [-next_direction(2), next_direction(1)];
            
            % Deslocando os pontos em ambas as direções para a próxima semi-reta
            next_p1_new = pDP(i+1,:) + alpha * robot_radius * next_normal;
            next_p2_new = pDP(i+2,:) + alpha * robot_radius * next_normal;
            
            % Calculating the equation of the straight line for the next shifting segment
            next_m = (next_p2_new(2) - next_p1_new(2)) / (next_p2_new(1) - next_p1_new(1));
            next_b = next_p1_new(2) - next_m * next_p1_new(1);
            
            % Calculating the intersection between the two straight lines
            x_intersect = (b - next_b) / (next_m - m);
            y_intersect = m * x_intersect + b;
            p_intersection = [x_intersect y_intersect];
        end
        
        % Adding the intersection points to the new polygon.
        if ~isnan(p_intersection(1)) && ~isnan(p_intersection(2))
            newPolygon = [newPolygon; p_intersection];
        end
        
        
    end
    
    
    % ---- Last segment:
    direction = pDP(numPoints,:) - pDP(numPoints-1,:);
    
    % Normalize the direction vector
    direction = direction / norm(direction);
    
    % Calculating the normal vector
    normal = [-direction(2), direction(1)];
    
    % Shifting the points in both directions
    p1_new = pDP(numPoints-1,:) + alpha * robot_radius * normal; 
    p2_new = pDP(numPoints,:) + alpha * robot_radius * normal; 
    
    newPolygon = [newPolygon; p2_new]; % We only need the second point (the last).    
    
    output = newPolygon;
end
% Converts the mesh structure defined in 'convex_hull' and the points in the mesh defined by 'hull_points'...
    % to a list of plane constraints (Ax + By + Cz = D) that the octree can take as a query paramter
    function [constraints] = convert_hull_to_constraints(convex_hull, hull_points, return_cell_insteadof_double)
    
        % init variables fro codegen
        num_triangles = 0;
        row = 0;
        triangle = [0, 0, 0];
        side1 = [0, 0, 0];
        side2 = [0, 0, 0];
    
%         num_triangles = numel(convex_hull(:,1)); % number of constraint triangles in the hull
        num_triangles = numel(convex_hull)/3; % changed from line above for codegen
%         non_unique_constraints = cell(num_triangles, 4);
        res_constraints = cell(num_triangles, 4); % changed from line above for codegen
        for row = 1:num_triangles
%             triangle = convex_hull(row, :);
            triangle = [convex_hull(row, 1), convex_hull(row, 2), convex_hull(row, 3)]; % changed from line above for codegen
            side1 = hull_points(triangle(3), :) - hull_points(triangle(1), :);
            side2 = hull_points(triangle(2), :) - hull_points(triangle(1), :);
            normal = cross(side1, side2);
%             normal = normal/norm(normal); % now its actually normalized to unit size
            d = dot(normal, hull_points(triangle(1), :));
%             non_unique_constraints{row, 1} = normal(1);
%             non_unique_constraints{row, 2} = normal(2);
%             non_unique_constraints{row, 3} = normal(3); 
%             non_unique_constraints{row, 4} = d;
            res_constraints{row, 1} = normal(1);
            res_constraints{row, 2} = normal(2);
            res_constraints{row, 3} = normal(3);
            res_constraints{row, 4} = d;
        end
%         unique_constraints = str2double(unique(string(non_unique_constraints), 'rows'));
        
        if return_cell_insteadof_double == 1
%             num_constraints = numel(unique_constraints(:, 1));
%             for u = 1:num_constraints
%                 constraints{u} = [unique_constraints(u, 1), unique_constraints(u, 2), unique_constraints(u, 3), unique_constraints(u, 4)];
%             end
            constraints = res_constraints;
        end
        if return_cell_insteadof_double == 0
%             constraints = unique_constraints;
            num_constraints = numel(res_constraints)/4; % changed from line above for codegen
            constraints = zeros(num_constraints, 4);
            for u = 1:num_constraints
                constraints(u, 1) = res_constraints{u, 1};
                constraints(u, 2) = res_constraints{u, 2};
                constraints(u, 3) = res_constraints{u, 3};
                constraints(u, 4) = res_constraints{u, 4};
            end
        end
    end
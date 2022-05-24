function mesh = giftwrap3d(points)
    % Returns the mesh pertaining to the convex hull of a set of points
% uses the gift wrapping alogirhtmn.
%
% Note - This algorithm had some issues with getting stuck in a loop repeating results
% a workaround was added to return after generating a dense enough mesh and sort/uniqueing the mesh. 
% When using this function make sure to check the output using trimesh to confirm
% that it's correct, for example: 
% trimesh(mesh, points(:,1), points(:,2), points(:,3), "FaceAlpha", 0.25, 'FaceColor', 'k')
%

num_points = numel(points(:,1));
MAX_STOP = 2*nchoosek(num_points,3);

first_point_id = uint32(0);
second_point_id = uint32(0);
third_point_id = uint32(0);

find_first_plane();

mesh = zeros(min(MAX_STOP, 10000), 3);
mesh(1,:) = [first_point_id, second_point_id, third_point_id];
% this is the index of the next row into mesh since the space is preallocated
index = 2;

coder.varsize('mesh', [inf 3], [1 0]);

number_of_edges = uint32(3);
% size_of_edges = uint32(3);
% preallocate an estimated max of unclosed edges at any given time (num_points*3)
unclosed_edges = zeros(3,num_points*3);
%coder.varsize('unclosed_edges', [3 inf], [0 1]);
unclosed_edges(:,[1,2,3]) = [[first_point_id; second_point_id; third_point_id] [second_point_id; third_point_id; first_point_id] [third_point_id; first_point_id; second_point_id]];

created_edges = false(num_points);
created_edges(first_point_id, second_point_id) = true;
created_edges(second_point_id, third_point_id) = true;
created_edges(third_point_id, first_point_id) = true;

p1 = 0;
p2 = 0;
p3 = 0;

vp2 = [0 0 0];
v_norm = [0 0 0];

% We keep going until all edges are closed
while number_of_edges > 0
   p1 = unclosed_edges(1, 1);
   p2 = unclosed_edges(2, 1);
   p3 = unclosed_edges(3, 1);
   %pop_edge(1);
   unclosed_edges(:, 1) = unclosed_edges(:, number_of_edges);
   number_of_edges = number_of_edges - 1;
   
   
   vp1 = points(p1, :);
   vp2 = points(p2, :);
   vp3 = points(p3, :);
   
  
%    plot_stuff = [vp1;vp2;vp3;vp1];
%    plot3(plot_stuff(:,1), plot_stuff(:,2), plot_stuff(:,3));
%    grid on;
%    pause()
   
   v_norm = cross(vp2-vp1, vp3-vp2);
   v_norm = v_norm./norm(v_norm);
   
   vba = vp2 - vp1;
   % this is the same as = points - vp1, but that doesn't work with matlab coder
   v_others = bsxfun(@minus, points, vp1);
   v_perp_c = (vp3-vp1) - (vba)*(((vp3-vp1)*(vba)')/((vba)*(vba)'));
   
   % Cross product to get normals
   % (New-A) X (B-A)
   v_other_norms = [
       (vba(3).*v_others(:,2)-vba(2).*v_others(:,3)) ...
       (vba(1).*v_others(:,3)-vba(3).*v_others(:,1)) ...
       (vba(2).*v_others(:,1)-vba(1).*v_others(:,2))
       ];
   
   angle_candidates = (v_other_norms*v_norm')./vecnorm(v_other_norms,2,2);
   
   wrong_side = v_others*v_perp_c' > 0;
   angle_candidates(wrong_side) = -2 - angle_candidates(wrong_side);
   angle_candidates([p1, p2, p3]) = -inf;
   
   % ----------------
   % Zach added 2020-09-09
   % if there aren't any positive angles we want the most negative angle not the max()
   % so take the abs() of angle_candidates and return the [p1, p2, p3] angles back to -inf
   if ~any(angle_candidates(angle_candidates>0))
       angle_candidates = abs(angle_candidates);
       angle_candidates([p1, p2, p3]) = -inf;
   end
   % ----------------
   
   [max_val, p_new_double] = max(angle_candidates);
   % ---------------
   % Rylan added 2021-09-20
   % breaks angle ties correctly
   if sum(angle_candidates == max_val) > 1
        top_candidates = find(angle_candidates == max_val);
        for cand = 1:length(top_candidates)
            if (~edge_exists(top_candidates(cand), p1) || edge_unclosed(top_candidates(cand), p1)) && ...
                    (~edge_exists(top_candidates(cand), p2) || edge_unclosed(top_candidates(cand), p2))
                p_new_double = top_candidates(cand);
                break
            end
        end
    end
    p_new = uint32(p_new_double);
%    
%    v_new = points(p_new, :);
%    plot3([vp1(1) v_new(1)], [vp1(2) v_new(2)], [vp1(3) v_new(3)]); 
%    plot3([vp2(1) v_new(1)], [vp2(2) v_new(2)], [vp2(3) v_new(3)]); 
   
   % mesh(end+1, :) = [p1 p_new p2];
    mesh(index,:) = [p1 p_new p2];
    index = index + 1;
   % If you want to view the CH object uncomment the following line
%    trimesh(mesh(1:index-1,:), points(:,1), points(:,2), points(:,3), "FaceAlpha", 0.25, 'FaceColor', 'k')
   %push_edge(p1, p_new, p2);
    edge_popped = false;
    for ee = 1:number_of_edges
           if (unclosed_edges(1, ee) == p1 && unclosed_edges(2, ee) == p_new) || ...
                   (unclosed_edges(2, ee) == p1 && unclosed_edges(1, ee) == p_new)
               %pop_edge(ee);
               unclosed_edges(:, ee) = unclosed_edges(:, number_of_edges);
               number_of_edges = number_of_edges - 1;
               edge_popped = true;
               break
           end
    end
    if ~edge_popped
        % New edge!
        number_of_edges = number_of_edges + 1;
        unclosed_edges(:, number_of_edges) = [p1;p_new;p2];
        created_edges(p1, p_new) = true;
    end
   
    %push_edge(p_new, p2, p1);
    edge_popped = false;
    for ee = 1:number_of_edges
           if (unclosed_edges(1, ee) == p_new && unclosed_edges(2, ee) == p2) || ...
                   (unclosed_edges(2, ee) == p_new && unclosed_edges(1, ee) == p2)
               %pop_edge(ee);
               unclosed_edges(:, ee) = unclosed_edges(:, number_of_edges);
               number_of_edges = number_of_edges - 1;
               edge_popped = true;
               break
           end
    end
    if ~edge_popped
        % New edge!
        number_of_edges = number_of_edges + 1;
        unclosed_edges(:, number_of_edges) = [p_new;p2;p1];
        created_edges(p_new, p2) = true;
    end
   
   % ------------
   % Zach added this 2020-09-09
   % this is for when two angles are tied the algorithm will always choose the lower indexed one
   % causing the mesh to infinitely grow over the same correct CH but never acknowledging all the
   % edges are closed.  This will shut down the function when the number of mesh triangles passes
   % the 2 times the total number of possible combinations.  Many may be duplicates, but if the 
   % CH is not solved before this, it will not finish.
%    if index > MAX_STOP
%        number_of_edges = uint32(0);
%    end
end
mesh = mesh(1:index-1,:);

    function find_first_plane()
        [~, v1_double] = min(points(:, 1));
        % First we find the point with the minimum x
        first_point_id = uint32(v1_double);
        v1 = points(first_point_id, :);
        
        % this is the same as = points - v1, but that doesn't work with matlab coder
        translated_points = bsxfun(@minus, points, v1);
        % For second point find the one which has the least angle to y-z plane.
        dot_angles = -translated_points(:, 1)./vecnorm(translated_points,2,2);
        dot_angles(first_point_id) = -inf;
        
        [~, v2_double] = max(dot_angles);
        second_point_id = uint32(v2_double);
        v2 = points(second_point_id, :);
        
        on_cone = v2-v1;
        tangent_to_cone = cross(on_cone, on_cone-[1 0 0]);
        normal_to_cone = cross(tangent_to_cone, on_cone);
        normal_to_cone = normal_to_cone./norm(normal_to_cone);
        if normal_to_cone(1) > 0
            normal_to_cone = -normal_to_cone;
        end
        
        % Cross products to get normals
        % new-a x b-a
        other_normals = [
            translated_points(:,2)*on_cone(3)-translated_points(:,3)*on_cone(2), ...
            translated_points(:,3)*on_cone(1)-translated_points(:,1)*on_cone(3), ...
            translated_points(:,1)*on_cone(2)-translated_points(:,2)*on_cone(1)
        ];
    
        dot_angles2 = (other_normals*normal_to_cone')./vecnorm(other_normals,2,2);
        dot_angles2(first_point_id) = -inf;
        dot_angles2(second_point_id) = -inf;
        
        [~, v3_double] = max(dot_angles2);
        third_point_id = uint32(v3_double);
        v3 = points(third_point_id, :);
        
        normal_direction = cross(v2-v1, v3-v2);
        if normal_direction(1) > 0 % If this for some reason points out
            temp_id = third_point_id;
            third_point_id = second_point_id;
            second_point_id = temp_id;
        end
        
        
    end

    function exists = edge_exists(a, b)
        created_edges = created_edges;
        exists = created_edges(a, b) == true || created_edges(b, a) == true;
    end

    function open = edge_unclosed(a, b)
        unclosed_edges = unclosed_edges;
        for ee = 1:number_of_edges
           if (unclosed_edges(1, ee) == a && unclosed_edges(2, ee) == b) || ...
                   (unclosed_edges(2, ee) == a && unclosed_edges(1, ee) == b)
               open = true;
               return
           end
        end
        open = false;
    end
end

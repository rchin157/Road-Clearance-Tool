classdef mocttree < handle
    %MOCTTREE An oct tree container class
    % Implemented using MEX functions for speed
    % Wraps around the unsafe C functions
    
    properties (Access = private)
        tree_ptr = uint64(0);
    end
    
    methods
        function obj = mocttree(points)
            %MOCTTREE Construct an instance of this class
            % Constructs an octree from an 3xM matrix of points or Mx3 matrix.
            % For a 3x3 its assumed to be 3xM
            points = double(points);
            if ~ismatrix(points)
                error('Not a matrix')
            end
            
            if (size(points, 1) ~= 3)
                if size(points, 2) == 3
                    points = points';
                else
                    error('Invalid dimensions')
                end
            end
            
            
            % Points is now formatted like this
            % [ x1 x2 x3 ... ]
            % [ y1 y2 y3 ... ]
            % [ z1 z2 z3 ... ]
            
            min_pointz = min(points, [], 2);
            max_pointz = max(points, [], 2);
            
            if (isempty(min_pointz))
                min_pointz = [0 0 0];
            end
            
            if (isempty(max_pointz))
                max_pointz = [1 1 1];
            end
            
            obj.tree_ptr = octtrees.createfreemoct(points, min_pointz, max_pointz);
        end
        
        function num_points = query_rect_count(obj, point1, point2)
            % Query points inside a rectangular area given as a bound
            % between two points
            point1 = double(point1);
            point2 = double(point2);
            
            if ~isvector(point1) || ~isvector(point2) || length(point1) ~= 3 || length(point2) ~= 3
                error('Bad inputs, the inputs must be vectors of length 3');
            end
            
            if isrow(point1)
                point1 = point1';
            end
            if isrow(point2)
                point2 = point2';
            end
            
            min_pointz = min([point1 point2], [], 2);
            max_pointz = max([point1 point2], [], 2);
            
            % Hand coded the constraints
            constrain_mat = [ 1, -1, 0,  0, 0,  0; ...
                              0,  0, 1, -1, 0,  0; ...
                              0,  0, 0,  0, 1, -1; ...
                min_pointz(1), -max_pointz(1), ...
                min_pointz(2), -max_pointz(2), ...
                min_pointz(3), -max_pointz(3) ];
            
            num_points = octtrees.query_count_moct(obj.tree_ptr, constrain_mat);
        end
        
        function point_indexes = query_rect_indexs(obj, point1, point2)
            % Query points inside a rectangular area given as a bound
            % between two points
            point1 = double(point1);
            point2 = double(point2);
            
            if ~isvector(point1) || ~isvector(point2) || length(point1) ~= 3 || length(point2) ~= 3
                error('Bad inputs, the inputs must be vectors of length 3');
            end
            
            if isrow(point1)
                point1 = point1';
            end
            if isrow(point2)
                point2 = point2';
            end
            
            
            min_pointz = min([point1 point2], [], 2);
            max_pointz = max([point1 point2], [], 2);
            
            % Hand coded the constraints
            constrain_mat = [ 1, -1, 0,  0, 0,  0; ...
                0,  0, 1, -1, 0,  0; ...
                0,  0, 0,  0, 1, -1; ...
                min_pointz(1), -max_pointz(1), ...
                min_pointz(2), -max_pointz(2), ...
                min_pointz(3), -max_pointz(3) ];
            
            point_indexes = octtrees.query_index_moct(obj.tree_ptr, constrain_mat);
        end
        
        function num_points = query_planes_count(obj, constraints)
            % Query points inside a region given by a number of constraints
            % constraints are planes with the equation
            % ax + by + cz >= d
            %
            % Constraints is a 4xN or Nx4 Matrix of doubles
            % [ a1 a2 a3 ... ]
            % [ b1 b2 b3 ... ]
            % [ c1 c2 c3 ... ]
            % [ d1 d2 d3 ... ]
            %
            % If constraits is 4x4 its assumed to be 4xN
            
            constraints = double(constraints);
            
            if size(constraints, 1) ~= 4
                if size(constraints, 2) == 4
                    constraints = constraints';
                else
                    error('Bad inputs size, must be a 4xN or Nx4  Matrix');
                end
            end
            
            num_points = octtrees.query_count_moct(obj.tree_ptr, constraints);
        end
        
        function point_indexes = query_planes_index(obj, constraints)
            % Query points inside a region given by a number of constraints
            % constraints are planes with the equation
            % ax + by + cz >= d
            %
           % Constraints is a 4xN or Nx4 Matrix of doubles
            % [ a1 a2 a3 ... ]
            % [ b1 b2 b3 ... ]
            % [ c1 c2 c3 ... ]
            % [ d1 d2 d3 ... ]
            %
            % If constraits is 4x4 its assumed to be 4xN
            
            constraints = double(constraints);
            
            if size(constraints, 1) ~= 4
                if size(constraints, 2) == 4
                    constraints = constraints';
                else
                    error('Bad inputs size, must be a 4xN or Nx4  Matrix');
                end
            end
            
            point_indexes = octtrees.query_index_moct(obj.tree_ptr, constraints);
        end
        
        function num_points = query_planes_count_par(obj, cell_constraints)
            % Query points inside a region given by a number of constraints
            % does it in parallel using a cell array of constraints
            
            % constraints are planes with the equation
            % ax + by + cz >= d
            %
            % Constraints is a 4xN or Nx4 Matrix of doubles
            % [ a1 a2 a3 ... ]
            % [ b1 b2 b3 ... ]
            % [ c1 c2 c3 ... ]
            % [ d1 d2 d3 ... ]
            %
            % If constraits is 4x4 its assumed to be 4xN
            
            if (isempty(cell_constraints))
                num_points = double.empty(0,1);
                return;
            end
            
            % Apply a check and fix to each cell_constraint
            cell_constraints = cellfun(@checkcons, cell_constraints);
            num_points = octtrees.query_count_moct_par(obj.tree_ptr, cell_constraints);
            
            function cons_double = checkcons(cons_double)
                cons_double = double(cons_double);

                if size(cons_double, 1) ~= 4
                    if size(cons_double, 2) == 4
                        cons_double = cons_double';
                    else
                        error('Bad inputs size, must be a 4xN or Nx4  Matrix');
                    end
                end
                
                cons_double = {cons_double};
            end
        end
        
        function num_points = query_planes_count_par_lim(obj, cell_constraints, limit)
            % Query points inside a region given by a number of constraints
            % does it in parallel using a cell array of constraints
            
            % constraints are planes with the equation
            % ax + by + cz >= d
            %
            % Constraints is a 4xN or Nx4 Matrix of doubles
            % [ a1 a2 a3 ... ]
            % [ b1 b2 b3 ... ]
            % [ c1 c2 c3 ... ]
            % [ d1 d2 d3 ... ]
            %
            % If constraits is 4x4 its assumed to be 4xN
            
            if (isempty(cell_constraints))
                num_points = double.empty(0,1);
                return;
            end
            
            % Apply a check and fix to each cell_constraint
            for i = 1:numel(cell_constraints)
               cell_constraints{i} = double(cell_constraints{i}); 
               if size(cell_constraints{i}, 1) ~= 4
                    if size(cell_constraints{i}, 2) == 4
                        cell_constraints{i} = cell_constraints{i}';
                    else
                        error('Bad inputs size, must be a 4xN or Nx4  Matrix');
                    end
                end
            end
            
            num_points = octtrees.query_count_moct_par_lim(obj.tree_ptr, cell_constraints, uint64(limit));
        end
        
        function delete(obj)
            % Delete the tree, and the underlying object
            if(obj.tree_ptr ~= 0)
                octtrees.createfreemoct(obj.tree_ptr);
            end
        end
    end
end


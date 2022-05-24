function [road_points, forwards, leftwards, upwards] = camera_path_magic(las_struct, traj)
%CAMERA_PATH_MAGIC Summary of this function goes here
% Performs magic to create a full frame for the vehical!
%
% Inputs:
%   las_data: Las points
%
%   traj: A structure with at least the following properties
%
% Outputs:
%
%
%% First get data along the vehicle path (scan angle = 0)

in = las_struct.scan_angle_rank == 0;
xyz_gpstime_id = [las_struct.x(in) las_struct.y(in) las_struct.z(in) las_struct.gps_time(in) las_struct.point_source_ID(in)];
scanners = unique(xyz_gpstime_id(:, 5)); % Find out how many scanners were on the vehicle
num_scanners = numel(scanners);
split_raw_road_path = cell(1, num_scanners); % Split the data by scanner

for i = 1:num_scanners
    % Sorted by GPS time (column 4)
    % Split raw road path is x y z gpstime
    split_raw_road_path{i} = sortrows( ...
        xyz_gpstime_id(xyz_gpstime_id(:, 5) == scanners(i), 1:4), 4);
end

%% Now we want to create a smooth path (which is road points)

% This is actually a fairly complex thing to try to do! Read up on the
% coastline paradox to see what I mean!

% First steps are to resample the data from our scanners, its currently in
% the time domain and is impossible to work with!

% Resample to the highest sensor sample rate
[num_resample_points, which] = max(cellfun('size', split_raw_road_path, 1)); % Get the one which has the most points
traj_times = split_raw_road_path{which}(:, 4);  % Times is column 4

% Now a bunch of complex smoothing happens which deals with the mess of
% noisy data!
road_points = zeros(num_resample_points, 3);
for i = 1:num_scanners
    road_points = road_points + magic_smooth(split_raw_road_path{i}(:, 4), split_raw_road_path{i}(:, 1:3), traj_times, 0.5);
    % Its a resampling and smoothing function                ^ these times   ^ this data        to these times ^        ^ this window size for smoothing (time) 
end
road_points = road_points ./ num_scanners; % We added the results, so average them out

distances = [0; cumsum(vecnorm(diff(road_points, 1),2,2))];
% Traj time gave some distances, both are monotonically increasing

total_distance = distances(end);
total_points = 1 + floor(total_distance/traj.point_density);

% Resample the points by using the distances we got to the distances we
% need to be at.
[~, index_unique] = unique(distances);
road_points = interp1(distances(index_unique), road_points(index_unique, :), linspace(0, total_distance, total_points)', 'linear');

%
% for i = 1:num_scanners
%     % Linearly resample the original points to be evenly spaced in time,
%     road_points = road_points + interp1(split_raw_road_path{i}(:,1), split_raw_road_path{i}(:,2:4), traj_times, 'linear');
%     %                                   ^                            ^                              ^
%     % Resample from        sampled at these times                   these points        to sampled at these times
% end
% road_points = road_points ./ num_scanners; % We added the results, so average them out
%
% % Now we have a single path representing the vehicle path, but measuring
% % chordal distance is difficult since the data has noise.
%
% % First lets see how spaced apart the points are right now (should be
% % approximately constant since the car has roughly constant velocity)
% spacing = median(vecnorm(diff(road_points, 1), 2, 2));
%
% measure_traj_times = linspace(min_time, max_time, ceil(num_resample_points * spacing / traj.measure_density));
% measure_road_points = interp1(traj_times, road_points, measure_traj_times, 'linear');
% measure_distances = [0; cumsum(vecnorm(diff(measure_road_points, 1), 2, 2))];
%
% traj_distances = interp1(measure_traj_times, measure_distances, traj_times);
% total_distance = traj_distances(end);
%
% total_points = 1 + floor(total_distance/traj.point_density);
% distances = linspace(0, (total_points-1)*traj.point_density, total_points);
%
% % Lots of weird interpolations just happened, but they create a rigourous
% % result!
% % Note that the distances do not relate to the cummulative chordal sum!
% road_points = interp1(traj_distances, road_points, distances);

%% Find the forwards direction vectors
% This is done using magic with least squares

% Ten meters in either direction
window_index_size = ceil(10/traj.point_density);

% This is some very complicated highly vectorized code
% what it does is for every point p it considers a region window_index_size
% in either direction, then it computes the least squares slope over x, y,
% and z for those points.
% This gives x'(d), y'(d), z'(d) for every point! Which is in essense the
% heading!

% Part of the reason its so complicated is because it does some special
% things for points on the edge.

% Gets indexes to deal with points on the edge
all_indexes = (1:total_points)';
left_error = max(0, 1 - (all_indexes - window_index_size)); % Too far left
right_error = max(0, all_indexes + window_index_size - total_points); % Too far right
shifts = all_indexes + left_error - right_error;
initials = shifts - window_index_size;
indexs_to_compare = (initials + (0:(2*window_index_size)))';
points_per = size(indexs_to_compare, 1);

% Now get some sections
sections = reshape(road_points(indexs_to_compare, :), points_per, total_points, 3);
sections = permute(sections, [1, 3, 2]);

% Amazing enough you can't actually do multidimensional dot products in
% matlab. It doesn't do broadcasting!
D = sections.*(1:points_per)';
D = sum(D, 1);
S = sum(sections, 1);

% This is least squares slope times a fixed constant, its a kind of magic
% trick.
slopes = D - S*((points_per + 1)/2);
forwards = reshape(slopes, 3, total_points)';
forwards = forwards ./ vecnorm(forwards, 2, 2);

%% Find the upwards vectors

xyz = sortrows([las_struct.x las_struct.y las_struct.z], 1); % Sorted by X
upwards = zeros(size(road_points));

for i = 1:total_points
    pos_i = road_points(i, :);
    
    pos_min = pos_i - traj.floor_box_edge/2;
    pos_max = pos_i + traj.floor_box_edge/2;
    
    % Get within X
    nearby_points = xyz(row_lower_bound(xyz, pos_min(1), 1):row_upper_bound(xyz, pos_max(1),1), :);
    
    % Get within Y
    nearby_points = nearby_points(nearby_points(:,2) >= pos_min(2), :);
    nearby_points = nearby_points(nearby_points(:,2) <= pos_max(2), :);
    
    % Get Within Z
    nearby_points = nearby_points(nearby_points(:,3) >= pos_min(3), :);
    nearby_points = nearby_points(nearby_points(:,3) <= pos_max(3), :);
    
    if size(nearby_points, 1) > 10
        upwards(i, :) = affine_fit(nearby_points);
        if (upwards(i, 3)/norm(upwards(i,:))) < 0.9 % 25 degrees tilt, sanity check.
           disp("BAD ANGLES!!!")
           upwards(i, :) = [0 0 1];
        end
    else
        % If we didn't find a bunch of points to fit to
        % Unlikely, we normally get several
        % thousands
        upwards(i, :) = [0 0 1];
    end
    
end
upwards = (upwards .* sign(upwards(:, 3)))./vecnorm(upwards, 2, 2); % Make sure the normals point up!

%% Find the leftwards vector
leftwards = cross(upwards, forwards, 2);

    function new_rs = magic_smooth(ts, rs, new_ts, window_ts)
        % Does some magic to smooth a function given by rs and ts
        % ts is a column vector of times
        % rs is a matrix, the different columns are different variables
        % new_ts are the new times you would like the samples to be at
        % new_rs are the smoothed interpolants
        % window_ts is the size of the window to use
        
        new_rs = zeros(numel(new_ts), size(rs, 2));
        
        val_array = [ts rs];
        val_array = sortrows(val_array, 1);
        
        index_start = 1;
        index_end = 4;
        
        for i_s = 1:numel(new_ts)
            t_0 = new_ts(i_s);
            
            % Move up index_end (keep it from going off the end)
            while index_end < size(val_array, 1) && val_array(index_end, 1) < t_0 + window_ts
                index_end = index_end+1;
            end
            % Move up index_start (make sure there are at least 4 points)
            while index_start < index_end-3 && val_array(index_start, 1) < t_0 - window_ts
                index_start = index_start+1;
            end
            
            n = index_end-index_start+1;
            t_vec = val_array(index_start:index_end, 1) - t_0;
            t_vec2 = t_vec.*t_vec;
            r_vec = val_array(index_start:index_end, 2:end);
            
            st = sum(t_vec);
            st2 = sum(t_vec2);
            st3 = t_vec2'*t_vec;
            st4 = t_vec2'*t_vec2;
            
            sr = sum(r_vec, 1);
            srt = t_vec'*r_vec;
            srt2 = t_vec2'*r_vec;
            
            % Crammers Rule to explicity solve for c in a least squares
            % solution to a quadratic
            det_denom = st4*st2*n + st3*st*st2 + st2*st3*st - st2*st2*st2 - st3*st3*n - st4*st*st;
            det_num = st4.*st2.*sr + st3.*srt.*st2 + srt2.*st3.*st - srt2.*st2.*st2 - st3.*st3.*sr - st4.*srt.*st;
            
            new_rs(i_s, :) = det_num ./ det_denom;
        end
    end

    function row_index = row_lower_bound(total_matrix, value, column)
        %LOWER_BOUND Least i such that total_matrix(i, column) >= value or [] if none exists
        
        left = 0;
        right = size(total_matrix, 1)+1;
        
        while left < right-1
            n = ceil((left+right)/2);
            
            if total_matrix(n, column) >= value
                right = n;
            else
                left = n;
            end
        end
        
        if left < size(total_matrix, 1)
            row_index = right;
        else
            row_index = [];
        end
    end
    function row_index = row_upper_bound(total_matrix, value, column)
        %LOWER_BOUND Max i such that total_matrix(i, column) <= value or [] if none exists
        
        left = 0;
        right = size(total_matrix, 1)+1;
        
        while left < right-1
            n = floor((left+right)/2);
            
            if total_matrix(n, column) <= value
                left = n;
            else
                right = n;
            end
        end
        
        if right > 1
            row_index = left;
        else
            row_index = [];
        end
    end
end

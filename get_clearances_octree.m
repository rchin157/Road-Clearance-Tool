%% Clearance Analyzer
% Written by Rylan Chin, February 2022
% Last modified February 25, 2022
% This script will automatically compute clearance data and filter points
% of interest to be plotted. Past team members wrote and/or acquired most
% of the scripts in lib and octrees which have been modified to suit this
% project.
% This version does not employ the parallel computing toolbox.
%% Open the main las file
tic
[las_files, las_path] = uigetfile('*.las;*.laz', 'Please select the main point cloud', 'MultiSelect', 'off');
disp('Loading las file');

las_struct = extract_las_data({las_files}, las_path);
[~,header] = LAS2DATA(strcat(las_path, las_files), 'txyzicap');
toc

%% variables
tstart = tic;

target_plane_width = 0.1;
scanwidth = 10;  % in whatever unit the las file is in (ie 10m or 10ft)
scantiles = ceil(scanwidth/target_plane_width);
if mod(scantiles, 2) == 0
    scantiles = scantiles + 1;
end
middlescan = ceil(scantiles / 2);
observer_height = 3;
max_height = 20; % for whatever reason using different heights may cause issues with giftwrap, 20 seems to work, 15 had issues
max_side = 16;
min_pts = 3;
% filtering variables
candidate_buffer = 10; % in whatever unit the las file is in (ie 10m or 10ft)
candidate_padding = 4; % in whatever unit your file is in
% cloud preprocessing
sample_percent = 1; % A float greater than 0 and less than or equal to 1. Example, 0.25 will keep 25% of points. Supports 1, 0.5, 0.25, 0.125, etc. (Only halfings)
translate_pts = true;

%% downsample
fn = fieldnames(las_struct);
for k=1:numel(fn)
    las_struct.(fn{k}) = las_struct.(fn{k})(1:ceil(1/sample_percent):end, :);
end

%% translate pts
if translate_pts
    las_struct.x = las_struct.x - header.x_offset;
    las_struct.y = las_struct.y - header.y_offset;
    las_struct.z = las_struct.z - header.z_offset;
end

%% Create octree
% Sort by time!
disp('Building octree');
tic
[~, idx] = sort(las_struct.gps_time);
las_points = [las_struct.x(idx) las_struct.y(idx) las_struct.z(idx)];
las_octree = octtrees.mocttree(las_points);
toc

%% Calculate road points, observers, and targets
if las_struct.gps_time == 0
    error("No GPS time in LAS file :(, cannot be used to make a trajectory!");
end
if las_struct.scan_angle_rank == 0
    error("No scan angle rank in LAS file :(, cannot be used to make trajectory!");
end
disp('Constructing Trajectory and Headings');

tic
traj.point_density = target_plane_width; % Meters/feet/etc per point
traj.floor_box_edge = 2; % Meters/feet/etc

[road_points, forwards, leftwards, upwards] = camera_path_magic(las_struct, traj);
num_road_points = numel(road_points(:,1));

toc

disp('Determining observers and scan targets')
tic
% initialize lists of targets and their associated observer points
target_planes_right = cell(num_road_points, 4);
target_planes_left = cell(num_road_points, 4);
target_planes_up = cell(num_road_points, 4);
h_observer_points = cell(1, num_road_points);
v_observer_points = cell(1, num_road_points);
% measurements are calculated along a "scan line", offsets serve to create
% observers along this line from the initial road point
h_offsets = linspace(1, scantiles, scantiles) - middlescan;
v_offsets = linspace(1, scantiles, scantiles);
for i = 1:num_road_points
    % create observers for current road point
    h_observer_points{i} = h_offsets.'*[leftwards(i,1) leftwards(i,2) 0]*target_plane_width + (road_points(i,:)+observer_height*[0 0 1]);
    v_observer_points{i} = v_offsets.'*[0 0 1]*target_plane_width + road_points(i,:) + [0 0 1];
    % create scan targets based on observers
    [target_planes_right{i, 1}, target_planes_right{i, 2}, target_planes_right{i, 3}, target_planes_right{i, 4}] = ...
                    get_target_plane_corners(v_observer_points{i}, forwards(i,:), leftwards(i,:), ...
                                               target_plane_width, "right", max_height, max_side, scantiles);
    [target_planes_left{i, 1}, target_planes_left{i, 2}, target_planes_left{i, 3}, target_planes_left{i, 4}] = ...
                    get_target_plane_corners(v_observer_points{i}, forwards(i,:), leftwards(i,:), ...
                                               target_plane_width, "left", max_height, max_side, scantiles);
    [target_planes_up{i, 1}, target_planes_up{i, 2}, target_planes_up{i, 3}, target_planes_up{i, 4}] = ...
                    get_target_plane_corners(h_observer_points{i}, forwards(i,:), leftwards(i,:), ...
                                               target_plane_width, "up", max_height, max_side, scantiles);
end
toc

%% measure clearances
disp("measuring clearances")
tic
% clearance lists are 2d matrices where rows represent a scan line along
% the vehicle trajectory and columns are for each roadpoint
top_clearances = zeros(scantiles, num_road_points);
left_clearances = zeros(scantiles, num_road_points);
right_clearances = zeros(scantiles, num_road_points);
for i = 1:num_road_points
    for j = 1:scantiles
        % calculate vertical clearance for the current scantile
        target_corners = [target_planes_up{i,1}(j,:); target_planes_up{i,2}(j,:); target_planes_up{i,3}(j,:); target_planes_up{i,4}(j,:)];
        top_constraint = get_constraint(h_observer_points{i}(j,:), target_corners);
        top_pt_idxs = las_octree.query_planes_index(top_constraint);
        % filter out noise
        if length(top_pt_idxs) < min_pts
            top_clearances(j,i) = max_height;
        else
            top_pts = las_points(top_pt_idxs, :);
            if size(top_pts,1) == 1
                top_min = top_pts;
            else
                top_min = min(min(top_pts,[],3));
            end
            % top clearance is calculated as the vertical difference
            % between the lowest point found and the road point
            top_z = top_min(3);
            bot_z = h_observer_points{i}(j,3) - observer_height;
            top_clearances(j,i) = top_z - bot_z;
        end

        % calculate left clearance for the current scantile
        target_corners = [target_planes_left{i,1}(j,:); target_planes_left{i,2}(j,:); target_planes_left{i,3}(j,:); target_planes_left{i,4}(j,:)];
        left_constraint = get_constraint(v_observer_points{i}(j,:), target_corners);
        left_pt_idxs = las_octree.query_planes_index(left_constraint);
        if length(left_pt_idxs) < min_pts
            left_clearances(j,i) = max_side;
        else
            % side clearance is calculated as the distance between the
            % observer point and the closest point found
            left_pts = las_points(left_pt_idxs, :);
            left_dists = vecnorm(left_pts - v_observer_points{i}(j,:), 2, 2);
            left_clearances(j,i) = min(left_dists);
        end

        % calculate right clearance for the current scantile
        target_corners = [target_planes_right{i,1}(j,:); target_planes_right{i,2}(j,:); target_planes_right{i,3}(j,:); target_planes_right{i,4}(j,:)];
        right_constraint = get_constraint(v_observer_points{i}(j,:), target_corners);
        right_pt_idxs = las_octree.query_planes_index(right_constraint);
        if length(right_pt_idxs) < min_pts
            right_clearances(j,i) = max_side;
        else
            right_pts = las_points(right_pt_idxs, :);
            right_dists = vecnorm(right_pts - v_observer_points{i}(j,:), 2, 2);
            right_clearances(j,i) = min(right_dists);
        end
    end
end
toc

%% initial plot
% simple plot of the raw data in its entirety
side_clearance_plot_height = 1; % in whatever units the las file is in
Xaxis = linspace(0, num_road_points*traj.point_density-1, num_road_points);

subplot(3, 1, 1);
plot(Xaxis, top_clearances(middlescan,:))
plotlim = max(floor(min(top_clearances(middlescan,:))-1), 0);
ylim([plotlim inf])
title("Top Clearance")

subplot(3, 1, 2);
plot(Xaxis, left_clearances(side_clearance_plot_height/target_plane_width,:))
plotlim = max(floor(min(left_clearances(side_clearance_plot_height/target_plane_width,:))-1), 0);
ylim([plotlim inf])
title("Left Clearance")

subplot(3, 1, 3);
plot(Xaxis, right_clearances(side_clearance_plot_height/target_plane_width,:))
plotlim = max(floor(min(right_clearances(side_clearance_plot_height/target_plane_width,:))-1), 0);
ylim([plotlim inf])
title("Right Clearance")

warning('off','MATLAB:MKDIR:DirectoryExists')
mkdir('out')
mkdir(['out/' las_files])
saveas(gcf,['out/' las_files '/alldataraw.png'])

%% filter bridge candidates
% filters for contiguous segments of interest using top clearance
% lots of overhanging obstructions may yield worse predictions
disp('Filtering For Candidates')
tic
avg_clear = mean(top_clearances(middlescan,:)); % the edge case of the entire las file being in a tunnel might break this, simply change to a fixed value greater than the ceiling of the tunnel to fix
candidates = cell(1);
cind = 1;
buffer = 0;
prev_state = 0;
bridgemax = 0; % used later to make plots look nicer
for i = 1:num_road_points
    val = top_clearances(middlescan,i);
    if val < avg_clear
        if val > bridgemax
            bridgemax = val;
        end
        % captures the indices of the candidates_padding units worth
        % of indices
        lim_low = max([i-(candidate_padding/traj.point_density) 1]);
        lim_high = min([i+(candidate_padding/traj.point_density) num_road_points]);
        if prev_state == 0
            candidates{1,cind} = linspace(lim_low, lim_high, lim_high-lim_low+1);
        else
            candidates{1,cind} = union(candidates{1,cind}, linspace(lim_low, lim_high, lim_high-lim_low+1));
        end
        prev_state = 1;
        buffer = 0;
    else
        % buffer attempts to prevent starting a new candidate prematurely 
        if buffer == candidate_buffer/traj.point_density && prev_state == 1
            cind = cind+1;
            buffer = 0;
            prev_state = 0;
        end
        buffer = buffer+1;
    end
end
toc

%% contour plots
disp('Generating Figures')
tic
% generate plots for each bridge candidate
toppad = 2;
yrange = ceil(bridgemax+toppad);
side_clearance_plot_height = 1; % in whatever units the las file is in
figure
for i = 1:length(candidates)
    % contour plots
    mkdir(['out/' las_files '/candidate' int2str(i)])
    distmarkers = cell(1,floor(length(candidates{i})*traj.point_density));
    for j = 1:length(distmarkers)
        ci = floor(j/traj.point_density)-(floor(1/traj.point_density)-1);
        distmarkers{j} = sprintf('%d+%03d',floor(candidates{i}(ci)*traj.point_density/1000),rem(floor(candidates{i}(ci)*traj.point_density),1000));
    end
    % x forward y across top clearance plot
    [X, Y] = meshgrid(linspace(0, length(candidates{i})*traj.point_density-1, length(candidates{i})), linspace(0, scantiles*target_plane_width-1, scantiles));
    contourf(X, Y, top_clearances(:,candidates{i}));
    title('Top Clearance')
    xlabel('Forward')
    ylabel('Right')
    set(gca,'xtick',0:2:floor(length(candidates{i})*traj.point_density)-1,'xticklabel',{distmarkers{1:2:end}});
    cb = colorbar();
    ylabel(cb, 'Clearance');
    saveas(gcf,['out/' las_files '/candidate' int2str(i) '/ceilingview_at_' int2str(candidates{i}(1)*target_plane_width) '_units.png'])
    % x forward y bot to top left clearance plot
    [X, Y] = meshgrid(linspace(0, length(candidates{i})*traj.point_density-1, length(candidates{i})), linspace(0, scantiles*target_plane_width-1, scantiles));
    contourf(X, Y, left_clearances(:,candidates{i}));
    ylim([0 min(yrange, scanwidth-1)])
    title('Left Clearance')
    xlabel('Forward')
    ylabel('Up')
    set(gca,'xtick',0:2:floor(length(candidates{i})*traj.point_density)-1,'xticklabel',{distmarkers{1:2:end}});
    cb = colorbar();
    ylabel(cb, 'Clearance');
    saveas(gcf,['out/' las_files '/candidate' int2str(i) '/leftview_at_' int2str(candidates{i}(1)*target_plane_width) '_units.png'])
    % x forward y bot to top right clearance plot
    [X, Y] = meshgrid(linspace(0, length(candidates{i})*traj.point_density-1, length(candidates{i})), linspace(0, scantiles*target_plane_width-1, scantiles));
    contourf(X, Y, right_clearances(:,candidates{i}));
    ylim([0 min(yrange, scanwidth-1)])
    title('Right Clearance')
    xlabel('Forward')
    ylabel('Up')
    set(gca,'xdir','reverse')
    set(gca,'YAxisLocation','right')
    set(gca,'xtick',0:2:floor(length(candidates{i})*traj.point_density)-1,'xticklabel',{distmarkers{1:2:end}});
    cb = colorbar();
    ylabel(cb, 'Clearance');
    saveas(gcf,['out/' las_files '/candidate' int2str(i) '/rightview_at_' int2str(candidates{i}(1)*target_plane_width) '_units.png'])
    
    % raw clearance plots
    X = linspace(0, length(candidates{i})*traj.point_density-1, length(candidates{i}));
    % middle top
    Y = top_clearances(middlescan, candidates{i});
    plot(X,Y)
    plotlim = max(floor(min(top_clearances(middlescan,candidates{i}))-1), 0);
    ylim([plotlim inf])
    title("Top Clearance")
    xlabel('Forward')
    ylabel('Clearance')
    saveas(gcf,['out/' las_files '/candidate' int2str(i) '/topclearance_at_' int2str(candidates{i}(1)*target_plane_width) '_units.png'])
    % left and right
    Y1 = left_clearances(side_clearance_plot_height/target_plane_width, candidates{i});
    Y2 = right_clearances(side_clearance_plot_height/target_plane_width, candidates{i});
    plot(X,Y1)
    hold on
    plot(X,Y2)
    hold off
    plotlimL = max(floor(min(left_clearances(middlescan,candidates{i}))-1), 0);
    plotlimR = max(floor(min(right_clearances(middlescan,candidates{i}))-1), 0);
    ylim([min(plotlimR,plotlimL) inf])
    title("Side Clearance")
    xlabel('Forward')
    ylabel('Clearance')
    legend('Left','Right')
    saveas(gcf,['out/' las_files '/candidate' int2str(i) '/sideclearance_at_' int2str(candidates{i}(1)*target_plane_width) '_units.png'])
end
toc
close all
disp('Complete!')
toc(tstart)

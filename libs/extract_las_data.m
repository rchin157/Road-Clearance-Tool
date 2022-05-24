function las_struct = extract_las_data(las_file_name, las_file_path)
%EXTRACT_LAS_DATA Helper function which goes through and loads in
%everything! It packs it into a las_struct which contains everything.
% I should definitely move this to its own function!

las_struct = struct();

% First just load all of the individual files
individual_files_structs = cell(numel(las_file_name), 1);
for i = 1:numel(las_file_name)
   fprintf('Loading %s\n', las_file_name{i});
   [~, individual_files_structs{i}] = las2mat(['-i "' fullfile(las_file_path, las_file_name{i}) '"']);
end

% Find out what fields are common to all of them, while we are at it get
% the total number of points.
common_fields = fieldnames(individual_files_structs{1});
total_points = numel(individual_files_structs{1}.(common_fields{1}));
for i = 2:numel(individual_files_structs)
    common_fields = intersect(common_fields, fieldnames(individual_files_structs{i}));
    total_points = total_points + numel(individual_files_structs{i}.(common_fields{1}));
end

% Only a few fields are useful, so we maintain only those!
common_fields = intersect(common_fields, {'x', 'y', 'z', 'gps_time', 'point_source_ID', 'scan_angle_rank'});

% Irrelevant now
% Remove the attributes field, which has odd sizes.
% common_fields(strcmp(common_fields, 'attributes')) = [];

% Now for each field add it to las_struct, consolidating everything
% together
fprintf('Allocating memory!\n');
for i = 1:numel(common_fields)
    field_name = common_fields{i};
    las_struct.(field_name) = zeros(total_points, 1, class(individual_files_structs{1}.(field_name)));
end


fprintf('Consolidating points!\n');
done_points = 0;
for i = 1:numel(individual_files_structs)
    st_index = done_points + 1;
    done_points = done_points + numel(individual_files_structs{i}.(common_fields{1}));
    en_index = done_points;
    for j = 1:numel(common_fields)
        field_name = common_fields{j};
        las_struct.(field_name)(st_index:en_index) = individual_files_structs{i}.(field_name)(:);
    end
end
end
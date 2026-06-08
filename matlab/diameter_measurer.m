%% Simple diameter measurement (no mesh generation)

fruit_files = {
    'apple.stl'
    'orange.stl'
    'strawberry.stl'
    'peach.stl'
    'pear.stl'
};

fruit_names = {
    'apple'
    'orange'
    'strawberry'
    'peach'
    'pear'
};

fprintf('\n=== FRUIT DIAMETER MEASUREMENTS ===\n');

for i = 1:length(fruit_files)
    try
        % Load STL directly (no PDE mesh)
        tri = stlread(fruit_files{i});
        vertices = tri.Points;
        
        % Bounding box
        minVals = min(vertices);
        maxVals = max(vertices);
        
        % Max diameter
        diameter_x = maxVals(1) - minVals(1);
        diameter_y = maxVals(2) - minVals(2);
        diameter_max = max(diameter_x, diameter_y);
        
        % Convert mm to m
        diameter_m = diameter_max / 1000;
        
        fprintf('%s: %.4f m (%.1f mm)\n', fruit_names{i}, diameter_m, diameter_max);
    catch
        fprintf('%s: FAILED\n', fruit_names{i});
    end
end
%% MAIN SCRIPT — Multi-Object Fruit Picking Study
clear; clc;

% Load your gripper and fruit configs
grippers = load_gripper_configs();
fruits = load_fruit_models();

fprintf('=== FRUIT PICKING GRASP PLANNER ===\n');
fprintf('Testing %d fruits × %d grippers\n\n', length(fruits), length(grippers));

% Storage for all results
results = table();

%% LOOP: Fruit × Gripper
for fi = 1:length(fruits)
    fruit = fruits(fi);
    fprintf('\n--- FRUIT %d/%d: %s ---\n', fi, length(fruits), fruit.name);
    
    % Check if STL exists
    if ~isfile(fruit.stl_file)
        fprintf('  WARNING: %s not found — skipping\n', fruit.stl_file);
        continue;
    end
    
    % STEP A: Load and mesh the STL (your Sections 1-3)
    fprintf('  Loading STL...');
    try
        [vertices, faces] = load_and_mesh_stl(fruit.stl_file);
        fprintf(' Done (%d vertices, %d faces)\n', size(vertices,1), size(faces,1));
    catch
        fprintf(' FAILED\n');
        continue;
    end
    
    % STEP B: Multi-height slicing (your Sections 4-6)
    fprintf('  Slicing mesh...');
    [allLoops, zValues, numSlices] = slice_mesh_multiheight(vertices, faces);
    fprintf(' Done (%d slices)\n', numSlices);
    
    %% INNER LOOP: Test each gripper on this fruit
    for gi = 1:length(grippers)
        gripper = grippers(gi);
        fprintf('    Gripper %d/%d: %s ... ', gi, length(grippers), gripper.name);
        
        % STEP C: Set up gripper parameters (PARAMETERIZED!)
        params = struct();
        params.minGripSpan    = gripper.minGripSpan;
        params.maxGripSpan    = gripper.maxGripSpan;
        params.fingerWidth2D  = gripper.fingerWidth2D;
        params.fingerWidth3D  = gripper.fingerWidth3D;
        
        % STEP D: Analyze power grasp feasibility (your Section 8)
        [graspable_slices, sliceDia, fingerSlots] = ...
            analyze_power_grasp(allLoops, zValues, numSlices, params);
        
        % STEP E: Compute approach direction (your Section 9)
        if ~isempty(fingerSlots)
            S9 = compute_approach_direction(allLoops, zValues, fingerSlots, sliceDia);
            grasp_success = ~isempty(S9.centroid);
            num_regions = size(fingerSlots, 1);
            delta = mean(diff(zValues));
            best_region_height = (fingerSlots(1,2) - fingerSlots(1,1) + 1) * delta;
        else
            S9 = struct();
            grasp_success = false;
            num_regions = 0;
            best_region_height = 0;
        end
        
        % STEP F: Compute grasp quality metrics (your Section 12 logic)
        if grasp_success
            [epsilon_metric, Q1_metric, force_closure_passes] = ...
                compute_grasp_quality(S9, allLoops, zValues, params);
        else
            epsilon_metric = 0;
            Q1_metric = 0;
            force_closure_passes = false;
        end
        
        fprintf('Success=%d, Regions=%d, Epsilon=%.4f\n', ...
            grasp_success, num_regions, epsilon_metric);
        
        % STEP G: Save result to table
        results = [results; ...
            table({fruit.name}, {gripper.name}, grasp_success, num_regions, ...
                  best_region_height, epsilon_metric, Q1_metric, force_closure_passes, ...
                  'VariableNames', {'Fruit', 'Gripper', 'Success', 'NumRegions', ...
                                     'RegionHeight_m', 'Epsilon', 'Q1', 'ForceClosurePasses'})];
        
        % STEP H: Save Gazebo input (for later physics testing)
        if grasp_success
            gazebo_filename = sprintf('gazebo_test_%s_%s.mat', fruit.name, gripper.name);
            gazebo_input.S9 = S9;
            gazebo_input.params = params;
            gazebo_input.fruit_stl = fruit.stl_file;
            gazebo_input.gripper_urdf = gripper.URDF_model_name;
            save(gazebo_filename, 'gazebo_input');
            fprintf('      → Saved: %s\n', gazebo_filename);
        end
        
    end  % End gripper loop
    
end  % End fruit loop

%% Save results
writetable(results, 'grasp_planning_results.csv');
fprintf('\n=== RESULTS SAVED ===\n');
disp(results);

%% Generate comparative plots
generate_comparative_plots(results, grippers, fruits);
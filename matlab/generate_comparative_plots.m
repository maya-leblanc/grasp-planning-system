function generate_comparative_plots(results, grippers, fruits)

nGrippers = length(grippers);
nFruits = length(fruits);

figure('Name', 'Comparative Analysis', 'Position', [100 100 1200 800]);

% --- Plot 1: Success Rate Heatmap ---
subplot(2,3,1);
success_matrix = zeros(nFruits, nGrippers);
for f = 1:nFruits
    for g = 1:nGrippers
        idx = find(strcmp(results.Fruit, fruits(f).name) & ...
                   strcmp(results.Gripper, grippers(g).name));
        if ~isempty(idx)
            success_matrix(f, g) = results.Success(idx);
        end
    end
end
imagesc(success_matrix);
colormap([1 0.3 0.3; 0.3 1 0.3]);
xlabel('Gripper'); ylabel('Fruit');
set(gca, 'XTick', 1:nGrippers, 'XTickLabel', {grippers.name}, ...
          'YTick', 1:nFruits, 'YTickLabel', {fruits.name});
title('Success Rate (Red=Fail, Green=Pass)');
colorbar('Ticks', [0 1], 'TickLabels', {'Fail', 'Pass'});

% --- Plot 2: Number of Regions ---
subplot(2,3,2);
region_matrix = zeros(nFruits, nGrippers);
for f = 1:nFruits
    for g = 1:nGrippers
        idx = find(strcmp(results.Fruit, fruits(f).name) & ...
                   strcmp(results.Gripper, grippers(g).name));
        if ~isempty(idx)
            region_matrix(f, g) = results.NumRegions(idx);
        end
    end
end
imagesc(region_matrix);
colorbar;
xlabel('Gripper'); ylabel('Fruit');
set(gca, 'XTick', 1:nGrippers, 'XTickLabel', {grippers.name}, ...
          'YTick', 1:nFruits, 'YTickLabel', {fruits.name});
title('Number of Graspable Regions');

% --- Plot 3: Epsilon (Quality) ---
subplot(2,3,3);
epsilon_matrix = zeros(nFruits, nGrippers);
for f = 1:nFruits
    for g = 1:nGrippers
        idx = find(strcmp(results.Fruit, fruits(f).name) & ...
                   strcmp(results.Gripper, grippers(g).name));
        if ~isempty(idx)
            epsilon_matrix(f, g) = results.Epsilon(idx);
        end
    end
end
imagesc(epsilon_matrix);
colorbar;
xlabel('Gripper'); ylabel('Fruit');
set(gca, 'XTick', 1:nGrippers, 'XTickLabel', {grippers.name}, ...
          'YTick', 1:nFruits, 'YTickLabel', {fruits.name});
title('Grasp Quality (Epsilon)');

% --- Plot 4: Success by Gripper ---
subplot(2,3,4);
gripper_success = zeros(nGrippers, 1);
for g = 1:nGrippers
    success_count = sum(results.Success(strcmp(results.Gripper, grippers(g).name)));
    total_count = sum(strcmp(results.Gripper, grippers(g).name));
    gripper_success(g) = success_count / max(total_count, 1);
end
bar(1:nGrippers, gripper_success*100, 'FaceColor', [0.3 0.7 1]);
ylabel('Success Rate (%)');
set(gca, 'XTick', 1:nGrippers, 'XTickLabel', {grippers.name});
title('Average Success by Gripper');
ylim([0 105]);
grid on;

% --- Plot 5: Success by Fruit ---
subplot(2,3,5);
fruit_success = zeros(nFruits, 1);
for f = 1:nFruits
    success_count = sum(results.Success(strcmp(results.Fruit, fruits(f).name)));
    total_count = sum(strcmp(results.Fruit, fruits(f).name));
    fruit_success(f) = success_count / max(total_count, 1);
end
bar(1:nFruits, fruit_success*100, 'FaceColor', [1 0.5 0.3]);
ylabel('Success Rate (%)');
set(gca, 'XTick', 1:nFruits, 'XTickLabel', {fruits.name});
title('Average Success by Fruit');
ylim([0 105]);
grid on;

% --- Plot 6: Quality vs Region Height ---
subplot(2,3,6);
scatter(results.RegionHeight_m, results.Epsilon, 80, ...
        double(results.Success), 'filled', 'MarkerEdgeColor', 'k');
xlabel('Region Height (m)');
ylabel('Epsilon (Quality)');
title('Quality vs Stability');
colormap(gca, [1 0.3 0.3; 0.3 1 0.3]);
grid on;

sgtitle('Comparative Analysis: Fruit × Gripper', 'FontSize', 14, 'FontWeight', 'bold');

end
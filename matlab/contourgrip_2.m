%% Introduction
% Authour: Maya LeBlanc
% Affiliation: McMaster University
% Start Date: August 12 2025
% Referal: Dr. Gary Bone, Department of Mechanical Engineering, Robotics
% and Manufacturing Automation Laboratory (RMAL), McMaster University
% Type: Grasp Planning Robotics Research

%% Configuration/Input; edit these before running

cfg.stlFile = fullfile(fileparts(mfilename('fullpath')), '..', 'stl', 'strawberry.stl');
% performs a series of operations to construct the full path to a file.

% mfilename('fullpath') is a function that returns the full path of current
% m-file executing the code without the .m extension
% fileparts(...) is a function that splits a full path into three
% components: path, filename, and extension.
% fileparts(mfilename('fullpath')) returns only the directory containing
% the current script
% alltogether this creates an absolute path pointing to apple.stl located
% in the stl folder, one level above the folder of the current script. 

cfg.fruitName = 'strawberry';           % used in CSV output

% defining a structure like cfg every time is necessary because MATLAB does
% not retain local variables automatically across runs. it ensures your
% structure is initialized, avoiding undefined variable errors. this
% behaviour is independednt of upcoming csv values, which only outputs
% current in-memory values. 

% Fruit deformability factor (0 = rigid, 0.08 = 8% diameter reduction under grip)
% Based on fruit firmness literature:
%   strawberry: 0.12  (very soft)
%   peach:      0.07  (moderately soft)
%   orange:     0.03  (firm skin)
%   apple:      0.02  (firm)
%   pear:       0.03  (firm at base where grasped)
cfg.deformFactor = 0.12;   % change per fruit

cfg.gripperName   = 'simplified';    % used in CSV output
cfg.numSlices     = 50;
cfg.meshHmax      = 5;               % finer mesh = smaller value

% meshHmax = 2, quality = very fine, speed = 40 mins
% meshHmax = 10, quality = medium, speed = ~1 min
% Hmax = 5 is a balance between speed and accuracy

cfg.stlUnits = 'mm';  

% set to 'mm' or 'm' based on how the STL was exported

% Robot hand parameters (meters)
cfg.minGripSpan   = 0.040;   % 40mm
cfg.maxGripSpan   = 0.090;   % 90mm
cfg.fingerWidth2D = 0.010;
cfg.fingerWidth3D = 0.020;

% Output CSV path
cfg.csvFile = 'grasp_results.csv';

%% Section 1: Load mesh

% read a 3D model stored in an STL file and extract its geometric data

% Read STL directly — bypass PDE mesh
[vertices, faces] = stlread_direct(cfg.stlFile);
% stlread_direct is a function (often a custom or third-party function)
% designed to read STL files directly, returning the raw verticies and
% faces of the 3D mesh. 

% vertices: an Nx3 matrix storing the coordinates of each vertex in 3D
% space. each row represents a single vertex with its x y z coordinates.

% faces: an Mx3 or MxN matrix storing the indices of vertices that from
% each triangular face of the mesh. each row contains indices refeering to
% rows in the vertices matrix. 

% Unit conversion: mm → m if needed

% Unit conversion
switch cfg.stlUnits
    case 'mm'
        fprintf('Converting mm → m\n');
        vertices = vertices / 1000;
    case 'm'
        % already in meters, no conversion needed
    otherwise
        error('cfg.stlUnits must be ''mm'' or ''m''');
end

% once you have vertices and faces, you can visualize or process the 3D
% model.

fprintf('Mesh loaded: %d vertices, %d triangles\n', size(vertices,1), size(faces,1));
fprintf('Bounds: X[%.4f %.4f]  Y[%.4f %.4f]  Z[%.4f %.4f] m\n', ...
    min(vertices(:,1)), max(vertices(:,1)), ...
    min(vertices(:,2)), max(vertices(:,2)), ...
    min(vertices(:,3)), max(vertices(:,3)));

% patch uses the vertices and faces to display a 3D surface of the object.

%% Section 2: Multi-Height Slicing

zMin     = min(vertices(:,3));
% find the minimum z vertex value
% vertices(:,3) means "all rows, column 3" -- : means "everything" along
% that dimension. This basically pulls out a single column vector
% containing just the z coordinate of every vertex in the mesh, ignoring x
% and y entirely. 
zMax     = max(vertices(:,3));
% min/max are functions that scan through to find smallest and largest
% number in z column per.
zValues  = linspace(zMin, zMax, cfg.numSlices);
% creates the list of heights where the slicing planes will go. It takes
% the bottom (zMin) and top (zMax) z valyes and generates a set of evenly
% spaced numbers between them dependent on the third parameter
% cfg.numSlices, which are the number of slices set. But the code
% later will cut a horizontal cross-section through the object.
% linspace spaces everything evenly.
delta    = mean(diff(zValues));
% the gap between consecutive slice heights. diff(zvalues) computes
% differences beteen neighbouring values. mean(...) averages them (mostly
% as a safe way to get the "spacing", even through for evenly spaced data
% they are all identical anyway. Just to make sure. 

% this spacing value matters later because it tells the code how much
% height in the real world one "slice" represents. It would be useful for
% converting a count of slices into an actual physical distance, like
% figuring out how many slices fit a 20mm gap.


% this loop goes through every slice height computed earlier and extracts
% the actual cross-sectional shape of the fruit at that height.

allLoops = cell(cfg.numSlices, 1); % sets up an empty container with one slot
% per slice = cell(one slice, has 1 slot available for that slice)
% a cell array in matlab is like a list that can hold different-shaped
% things in each slot. very useful here because each slide might produce a
% differnt number of points/loops)
% if numSlices = 50, you get 50 empty slots ready to be filled in

for i = 1:cfg.numSlices % loop that runs once per slice, using i to step 
    % through every height in zValues
    segments = sliceMesh(vertices, faces, zValues(i)); 
    % calls the sliceMesh function with 3 inputs: vertices = the Nx3 matrix 
    % of all the 3D points in the mesh, faces = the Mx3 matrix defining
    % which vertices form each triangle, and zValues(i) = the height of the
    % current slicing plane (a single number for a slice # of a fruit type)
    
    % segments = Kx6 matrix where each row is one line segment [x1 y1 z1 x2
    % y2 x2], the two endpoints of where the plane cut through one
    % triangle. K depends on how many triangles the plane intersects at
    % that height. 
    allLoops{i} = stitchSegments(segments);
    % returns a cell array of loops, where each loop is its own px3 matrix
    % of ordered 3d points forming one closed contour. stored in allLops{i}
    % using curly braces because allLoops is a cell array (needed since
    % each slice can have a different number of loops with different
    % numbers of points).
end

% Visualise all slices
figure('Name', 'Stacked Cross-Sections');
hold on; axis equal; grid on;
cmap = jet(cfg.numSlices);
for i = 1:cfg.numSlices
    for j = 1:numel(allLoops{i})
        lp = allLoops{i}{j};
        plot3(lp(:,1), lp(:,2), lp(:,3), '-', 'Color', cmap(i,:), 'LineWidth', 1.5);
    end
end
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Stacked Cross-Sections'); view(3);

%% Section 3: Grasp Feasibility Analysis

% Pass 1 — global diameter to find candidate finger slots
% captures sliceDia_rigid so we can compare rigid vs effective later
[sliceDia, sliceOK, fingerSlots, sliceDia_rigid] = analyzeGrasp(allLoops, zValues, delta, cfg);

% Pass 2 — if slots were found, compute approach direction then re-evaluate
% diameter specifically along the gripper's closing axis
if ~isempty(fingerSlots)
    % get approach direction from pass 1 slots
    S9_temp = computeApproachDirection(allLoops, zValues, fingerSlots, delta, cfg);

    % closing axis is perpendicular to approach direction in XY plane
    % if gripper approaches from direction [dx dy], it closes along [-dy dx]
    closingAxis = [-S9_temp.approachDir(2), S9_temp.approachDir(1)];

    % re-run analyzeGrasp using oriented diameter along closing axis
    % also captures updated rigid diameters measured along closing axis
    [sliceDia, sliceOK, fingerSlots, sliceDia_rigid] = analyzeGrasp(allLoops, zValues, delta, cfg, closingAxis);

    fprintf('Oriented diameter pass complete, closing axis: [%.3f %.3f]\n', closingAxis);
else
    % no slots found in pass 1 — skip pass 2
    % closingAxis is undefined so set it empty for downstream safety
    closingAxis = [];
    fprintf('No regions in pass 1, skipping oriented diameter pass.\n');
end

% print deformability summary so you can see the correction effect
fprintf('Deformability factor: %.0f%%\n', cfg.deformFactor*100);
fprintf('Max rigid diameter: %.1fmm\n', max(sliceDia_rigid(~isnan(sliceDia_rigid)))*1000);
fprintf('Max effective diameter: %.1fmm\n', max(sliceDia(~isnan(sliceDia)))*1000);

% Plot diameter profile
figure('Name', 'Grasp Feasibility');
subplot(1,2,1);
plotDiameterProfile(zValues, sliceDia, sliceOK, fingerSlots, cfg);

subplot(1,2,2);
plotMug3D(allLoops, fingerSlots, cfg.numSlices);
title('Graspable Regions (red)');

% Print results
printGraspResults(fingerSlots, zValues, delta, cfg);

%% Section 4: Gripper Approach Direction

if isempty(fingerSlots)
    fprintf('No graspable regions, skipping approach direction.\n');
    S9 = [];
else
    S9 = computeApproachDirection(allLoops, zValues, fingerSlots, delta, cfg);

    figure('Name', 'Approach Direction');
    hold on; axis equal; grid on;
    plotMug3D(allLoops, fingerSlots, cfg.numSlices);

    % Midpoint contour
    midLoop = allLoops{S9.midSliceIdx}{S9.lgIdx};
    plot3(midLoop(:,1), midLoop(:,2), midLoop(:,3), '-b', 'LineWidth', 2);
    plot3(S9.centroid(1), S9.centroid(2), S9.centroid(3), 'ko', ...
          'MarkerFaceColor','k','MarkerSize',10);

    arrowScale = norm(S9.preGraspPos(1:2) - S9.centroid(1:2));
    quiver3(S9.preGraspPos(1), S9.preGraspPos(2), S9.preGraspPos(3), ...
            S9.approachDir(1)*arrowScale, S9.approachDir(2)*arrowScale, 0, ...
            0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    plot3(S9.preGraspPos(1), S9.preGraspPos(2), S9.preGraspPos(3), ...
          'g^', 'MarkerFaceColor','g', 'MarkerSize',12);

    title('Section 9: Gripper Approach Direction'); view(3); hold off;
    fprintf('Approach dir: [%.3f %.3f %.3f]  Pre-grasp: (%.4f %.4f %.4f)\n', ...
        S9.approachDir, S9.preGraspPos);
end

%% Section 5: Uncertainty Margin Analysis

delta_u_values = [0, 0.0005, 0.001, 0.0015, 0.002, 0.003];
[slicesOK_per_delta, regionCount] = uncertaintyAnalysis( ...
    sliceDia, zValues, delta, delta_u_values, cfg);

figure('Name', 'Uncertainty Margin');
subplot(1,2,1);
imagesc(1:cfg.numSlices, delta_u_values*1000, slicesOK_per_delta);
colormap([1 1 1; 0.2 0.6 1]);
xlabel('Slice Index'); ylabel('\delta_u (mm)');
title('Slice Feasibility vs. Uncertainty');
colorbar('Ticks',[0 1],'TickLabels',{'Not OK','OK'});
set(gca,'YTick', delta_u_values*1000);

subplot(1,2,2);
bar(delta_u_values*1000, regionCount, 'FaceColor',[0.2 0.6 1],'EdgeColor','k');
xlabel('\delta_u (mm)'); ylabel('Valid Grasp Regions');
title('Region Count vs. Uncertainty'); grid on;

%% Section 6: Partial Mesh / Simulated Occlusion

occlusionFractions = [0, 0.10, 0.20, 0.30];
[regionCounts_occ, regionOK_occ] = occlusionTest( ...
    vertices, faces, zValues, cfg, sliceDia, delta);

figure('Name', 'Occlusion Tolerance');
nOcc = numel(occlusionFractions);
for oi = 1:nOcc
    subplot(1, nOcc, oi);
    hold on; grid on;
    plot(zValues, sliceDia, '-o', 'Color',[0.6 0.6 0.6], 'LineWidth',1, 'MarkerSize',3);
    yline(cfg.minGripSpan, '--r', 'LineWidth', 1.5);
    yline(cfg.maxGripSpan, '--g', 'LineWidth', 1.5);
    okIdx = find(regionOK_occ{oi});
    if ~isempty(okIdx)
        scatter(zValues(okIdx), sliceDia(okIdx), 40, 'filled', ...
                'MarkerFaceColor',[0.2 0.5 0.9]);
    end
    xlabel('Z (m)'); ylabel('Diameter (m)');
    title(sprintf('Occlusion: %.0f%%\n(%d region(s))', ...
          occlusionFractions(oi)*100, regionCounts_occ(oi)));
    ylim([0, max(sliceDia(~isnan(sliceDia)))*1.2]);
    hold off;
end
sgtitle('Section 11: Grasp Under Occlusion');

%% Section 7: Geometric Validation

runValidation(vertices, faces, zValues, allLoops, sliceDia, sliceDia_rigid, cfg, delta, closingAxis);

%% Section 8: Gripper Parameter Sweep 

% Based on first round of results (orange (86mm) + peach (79mm) strong grasps with
% 40-90mm gripper, apple (93 mm) + pear (96mm) too wide with only weak grasps at narrow
% ends, and strawberry (46 mm) too small and gripper cant reach), the sweep
% will answer: "what is the optimal gripper span for each fruit?"

% pass both sliceDia (effective) and sliceDia_rigid into the sweep
% sweep uses effective diameters to stay consistent with main analysis
[optMin, optMax, sweepTable] = gripperSpanSweep(sliceDia, sliceDia_rigid, zValues, delta, cfg);

% Plot sweep results
figure('Name', 'Gripper Span Sweep');
imagesc(sweepTable.maxSpans*1000, sweepTable.minSpans*1000, sweepTable.regionCounts);
colorbar; colormap(jet);
xlabel('Max Grip Span (mm)');
ylabel('Min Grip Span (mm)');
title(sprintf('Grasp Regions vs Gripper Span — %s', cfg.fruitName));
hold on;
plot(optMax*1000, optMin*1000, 'w*', 'MarkerSize', 15, 'LineWidth', 2);
hold off;

fprintf('\nOptimal gripper for %s: min=%.1fmm  max=%.1fmm\n', ...
    cfg.fruitName, optMin*1000, optMax*1000);

%% CSV Export for Gazebo

exportGraspCSV(S9, cfg);

fprintf('\nAll sections are complete.\n');

%% Local functions

% -------------------------------------------------------------------------
function [vertices, faces] = stlread_direct(filename)
% Read binary or ASCII STL directly without PDE toolbox
    fid = fopen(filename, 'rb');
    fread(fid, 80, 'uint8');        % header
    nTri = fread(fid, 1, 'uint32'); % number of triangles
    vertices = zeros(nTri*3, 3);
    faces    = zeros(nTri, 3);
    for i = 1:nTri
        fread(fid, 3, 'float32');   % normal vector (skip)
        v1 = fread(fid, 3, 'float32')';
        v2 = fread(fid, 3, 'float32')';
        v3 = fread(fid, 3, 'float32')';
        fread(fid, 1, 'uint16');    % attribute byte count
        idx = (i-1)*3 + 1;
        vertices(idx,:)   = v1;
        vertices(idx+1,:) = v2;
        vertices(idx+2,:) = v3;
        faces(i,:) = [idx, idx+1, idx+2];
    end
    fclose(fid);
end

% -------------------------------------------------------------------------
function segments = sliceMesh(vertices, faces, z0)
% Intersect triangular mesh with horizontal plane at height z0.
% Returns Nx6 array of segment endpoint pairs (each row = [pt1, pt2]).
    segments = [];
    for f = 1:size(faces,1)
        tv   = vertices(faces(f,:), :);
        zv   = tv(:,3);
        if (max(zv) < z0) || (min(zv) > z0) || all(zv == z0), continue; end
        pts = [];
        for e = [1 2; 2 3; 3 1]'
            p1 = tv(e(1),:);  p2 = tv(e(2),:);
            z1 = p1(3)-z0;    z2 = p2(3)-z0;
            if z1*z2 < 0
                t = z1/(z1-z2);
                pts(end+1,:) = p1 + t*(p2-p1); 
            elseif z1==0 && z2~=0
                pts(end+1,:) = p1;
            elseif z2==0 && z1~=0
                pts(end+1,:) = p2;
            end
        end
        if size(pts,1) == 2
            segments = [segments; pts]; 
        end
    end
end

% -------------------------------------------------------------------------
function loops = stitchSegments(segments)
% Chain unordered line segments into closed (or open) loops.
    loops     = {};
    tol       = 1e-6;
    while size(segments,1) >= 2
        loop = [segments(1,:); segments(2,:)];
        segments(1:2,:) = [];
        closed = false;
        while ~closed
            found = false;
            for j = 1:2:size(segments,1)-1
                s1 = segments(j,:);  s2 = segments(j+1,:);
                if     norm(s1 - loop(end,:)) < tol,  loop = [loop; s2];      segments(j:j+1,:)=[]; found=true; break;
                elseif norm(s2 - loop(end,:)) < tol,  loop = [loop; s1];      segments(j:j+1,:)=[]; found=true; break;
                elseif norm(s1 - loop(1,:))   < tol,  loop = [s2; loop];      segments(j:j+1,:)=[]; found=true; break;
                elseif norm(s2 - loop(1,:))   < tol,  loop = [s1; loop];      segments(j:j+1,:)=[]; found=true; break;
                end
            end
            if ~found, closed = true; end
        end
        loops{end+1} = loop; 
    end
end

% -------------------------------------------------------------------------
function [sliceDia, sliceOK, fingerSlots, sliceDia_rigid] = analyzeGrasp(allLoops, zValues, delta, cfg, closingAxis)
% analyzeGrasp — measures fruit diameter at each slice height and identifies
% vertical regions where a gripper can fit (finger slots).
%
% PASS 1 (no closingAxis): uses global maximum diameter in any direction.
% PASS 2 (closingAxis provided): uses diameter along gripper's closing axis only.
%
% Both passes apply a deformability correction (cfg.deformFactor) to model
% soft fruit compression under grip pressure. The rigid-body diameter is
% stored separately in sliceDia_rigid for comparison and reporting.
%
% INPUTS:
%   allLoops    — cell array of contour loops per slice (from stitchSegments)
%   zValues     — 1xN array of slice heights in meters
%   delta       — vertical spacing between slices in meters
%   cfg         — config struct containing:
%                   cfg.minGripSpan   — minimum gripper span (m)
%                   cfg.maxGripSpan   — maximum gripper span (m)
%                   cfg.fingerWidth3D — minimum finger slot height (m)
%                   cfg.deformFactor  — fruit compression fraction (0–1)
%                                       e.g. 0.07 = 7% diameter reduction
%   closingAxis — (optional) 1x2 unit vector [cx cy] — direction gripper
%                 fingers close along, perpendicular to approach direction
%
% OUTPUTS:
%   sliceDia       — Nx1 array of effective diameters after deformation (m)
%   sliceOK        — Nx1 logical, true where effective diameter fits gripper
%   fingerSlots    — Rx2 matrix of [startSlice, endSlice] for valid regions
%   sliceDia_rigid — Nx1 array of raw rigid-body diameters before deformation

    n              = numel(zValues);  % total number of slices
    sliceDia       = nan(n,1);        % effective diameter after deformation
    sliceDia_rigid = nan(n,1);        % raw rigid diameter before deformation
    sliceOK        = false(n,1);      % feasibility flag per slice

    % check if a closing axis was passed in (pass 2) or not (pass 1)
    % nargin counts how many input arguments were actually provided
    useOrientedDiameter = (nargin == 5) && ~isempty(closingAxis);

    for i = 1:n
        % skip empty slices — top and bottom of fruit have no intersection
        if isempty(allLoops{i}), continue; end

        % pick the largest contour loop at this height
        % cellfun applies size(L,1) to every loop and max picks the biggest
        % the ~ discards the max value itself, li is the index of that loop
        [~,li] = max(cellfun(@(L) size(L,1), allLoops{i}));
        lp     = allLoops{i}{li};   % Px3 matrix of contour points at slice i

        % compute raw rigid-body diameter
        try
            % convhull finds the outer boundary of the cross-section points
            % working in XY only (columns 1 and 2) since this is a horizontal slice
            k        = convhull(lp(:,1), lp(:,2));
            hull_pts = lp(k, 1:2);   % XY coordinates of convex hull points only

            if useOrientedDiameter
                % --- PASS 2: oriented diameter along gripper closing axis ---
                % the closing axis is perpendicular to the approach direction.
                % projecting each hull point onto this axis with a dot product
                % gives a scalar: how far along the closing direction that point sits.
                % width = furthest projection minus nearest projection.
                % this measures exactly what the gripper fingers would span.
                projections = hull_pts * closingAxis(:);
                % hull_pts is Px2, closingAxis(:) is 2x1 column vector
                % result is Px1 vector of scalar projections
                D = max(projections) - min(projections);

            else
                % --- PASS 1: global diameter (any direction) ---
                % find maximum pairwise distance between any two hull points.
                % this is the true geometric diameter regardless of direction.
                % used in pass 1 because we don't know approach direction yet.
                D = 0;
                for p1 = 1:size(hull_pts,1)
                    for p2 = p1+1:size(hull_pts,1)
                        d = norm(hull_pts(p1,:) - hull_pts(p2,:));
                        if d > D, D = d; end
                    end
                end
            end

        catch
            % fallback if convhull fails (e.g. fewer than 3 non-collinear points)
            % bounding box width is a rough but safe approximation
            D = max(max(lp(:,1))-min(lp(:,1)), max(lp(:,2))-min(lp(:,2)));
        end

        % store raw rigid-body diameter before any deformation correction
        sliceDia_rigid(i) = D;

        % apply deformability correction
        % soft fruits compress under grip pressure so their effective diameter
        % is smaller than the rigid mesh. cfg.deformFactor is the fraction of
        % diameter reduction:
        %   strawberry: 0.12 — very soft, significant compression
        %   peach:      0.07 — moderately soft
        %   orange:     0.03 — firm skin, minimal compression
        %   apple:      0.02 — firm, minimal compression
        %   pear:       0.03 — firm at base where grasped
        D_eff = D * (1 - cfg.deformFactor);

        % store effective diameter for use in all downstream analysis
        sliceDia(i) = D_eff;

        % feasibility check: does the effective diameter fit the gripper?
        % both conditions must be true — not too small to grip, not too wide
        sliceOK(i) = (D_eff >= cfg.minGripSpan) && (D_eff <= cfg.maxGripSpan);
    end

    % group consecutive feasible slices into finger slots
    % a slot is only valid if its vertical height >= fingerWidth3D (e.g. 20mm)
    % otherwise the finger physically cannot make contact over enough surface
    fingerSlots = [];
    inReg  = false;  % flag: are we currently inside a feasible region?
    rStart = 0;      % slice index where the current feasible region began

    for i = 1:n
        if sliceOK(i) && ~inReg
            % entering a new feasible region — record the starting slice
            inReg  = true;
            rStart = i;

        elseif ~sliceOK(i) && inReg
            % leaving a feasible region — check if it was tall enough
            % (i-rStart)*delta converts slice count to physical height in meters
            if (i-rStart)*delta >= cfg.fingerWidth3D
                % tall enough — save as a valid finger slot [startIdx, endIdx]
                fingerSlots(end+1,:) = [rStart, i-1]; 
            end
            inReg = false;
        end
    end

    % handle the edge case where a feasible region runs all the way to the
    % last slice without hitting an infeasible slice to trigger the save above
    if inReg && (n-rStart+1)*delta >= cfg.fingerWidth3D
        fingerSlots(end+1,:) = [rStart, n]; 
    end
end

% -------------------------------------------------------------------------
function S9 = computeApproachDirection(allLoops, zValues, fingerSlots, delta, cfg)
% Find widest angular gap in midpoint contour → approach direction.
    regionHeights = (fingerSlots(:,2) - fingerSlots(:,1) + 1) * delta;
    [~, bestR]    = max(regionHeights);
    bestStart     = fingerSlots(bestR,1);
    bestEnd       = fingerSlots(bestR,2);
    midIdx        = round((bestStart+bestEnd)/2);

    midLoops = allLoops{midIdx};
    if isempty(midLoops)
        for off = 1:3
            if midIdx+off <= numel(zValues) && ~isempty(allLoops{midIdx+off})
                midLoops = allLoops{midIdx+off};  break;
            end
        end
    end
    [~,lgIdx] = max(cellfun(@(L) size(L,1), midLoops));
    midLoop   = midLoops{lgIdx};

    cx = mean(midLoop(:,1));  cy = mean(midLoop(:,2));  cz = zValues(midIdx);

    angles        = atan2(midLoop(:,2)-cy, midLoop(:,1)-cx);
    angles_sorted = sort(angles);
    gaps          = [diff(angles_sorted); (2*pi - angles_sorted(end)) + angles_sorted(1)];
    [maxGap, gIdx] = max(gaps);

    if gIdx < numel(gaps)
        gapMidAngle = angles_sorted(gIdx) + gaps(gIdx)/2;
    else
        gapMidAngle = angles_sorted(end) + gaps(end)/2;
        if gapMidAngle > pi, gapMidAngle = gapMidAngle - 2*pi; end
    end

    approachDir  = [-cos(gapMidAngle), -sin(gapMidAngle), 0];
    approachDir  = approachDir / norm(approachDir);
    preGraspPos  = [cx, cy, cz] - approachDir * cfg.maxGripSpan * 1.5;

    S9.bestStart   = bestStart;
    S9.bestEnd     = bestEnd;
    S9.midSliceIdx = midIdx;
    S9.lgIdx       = lgIdx;
    S9.centroid    = [cx, cy, cz];
    S9.approachDir = approachDir;
    S9.preGraspPos = preGraspPos;
    S9.gapAngleDeg = rad2deg(gapMidAngle);
    S9.maxGapDeg   = rad2deg(maxGap);
end

% -------------------------------------------------------------------------
function [slicesOK_per_delta, regionCount] = uncertaintyAnalysis( ...
    sliceDia, zValues, delta, delta_u_values, cfg)

    n      = numel(zValues);
    nU     = numel(delta_u_values);
    slicesOK_per_delta = zeros(nU, n);
    regionCount        = zeros(nU, 1);

    for ui = 1:nU
        du   = delta_u_values(ui);
        mnU  = cfg.minGripSpan + du;
        mxU  = cfg.maxGripSpan - du;
        if mnU >= mxU, continue; end

        
        % also propagate uncertainty in the deformability estimate itself
        % deformFactor has ~±20% relative uncertainty from literature values
        % so effective diameter has additional uncertainty of D * deformFactor * 0.20
        % this adds to the gripper span uncertainty symmetrically
        deformUncertainty = mean(sliceDia(~isnan(sliceDia))) * cfg.deformFactor * 0.20;

        % combined uncertainty: positional + deformability
        % slices must be feasible even at worst-case combined uncertainty
        okU = (sliceDia >= mnU + deformUncertainty) & ...
            (sliceDia <= mxU - deformUncertainty) & ...
            ~isnan(sliceDia);
        slicesOK_per_delta(ui,:) = okU';  
        nReg=0; inReg=false; rStart=0;
        for i=1:n
            if okU(i)&&~inReg, inReg=true; rStart=i;
            elseif ~okU(i)&&inReg
                if (i-rStart)*delta>=cfg.fingerWidth3D, nReg=nReg+1; end
                inReg=false;
            end
        end
        if inReg && (n-rStart+1)*delta>=cfg.fingerWidth3D, nReg=nReg+1; end
        regionCount(ui) = nReg;

        fprintf('delta_u=%.1fmm (deform±%.1fmm) → [%.3f %.3f]m → %d region(s)\n', ...
            du*1000, deformUncertainty*1000, mnU, mxU, nReg);
    end
end

% -------------------------------------------------------------------------
function [regionCounts_occ, regionOK_occ] = occlusionTest( ...
    vertices, faces, zValues, cfg, sliceDia, delta)
% occlusionTest — re-runs grasp feasibility analysis at four occlusion levels
% by removing a fraction of mesh triangles from one side of the fruit,
% simulating partial visibility caused by leaves, branches, or neighbouring fruit.
%
% Deformability correction (cfg.deformFactor) is applied consistently here
% so occlusion results are comparable to the main analysis.
%
% INPUTS:
%   vertices  — Nx3 matrix of mesh vertex coordinates (meters)
%   faces     — Mx3 matrix of triangle vertex indices
%   zValues   — 1xN slice heights in meters
%   cfg       — config struct (minGripSpan, maxGripSpan, fingerWidth3D, deformFactor)
%   sliceDia  — not used directly here, kept for signature consistency
%   delta     — vertical spacing between slices in meters
%
% OUTPUTS:
%   regionCounts_occ — 4x1 array of valid grasp region counts per occlusion level
%   regionOK_occ     — 4x1 cell array of logical feasibility vectors per occlusion level

    occlusionFractions = [0, 0.10, 0.20, 0.30];  % 0%, 10%, 20%, 30% occlusion
    nOcc = numel(occlusionFractions);
    n    = numel(zValues);

    % identify which triangles are on the positive-X side of the fruit centroid
    % these are the triangles that will be progressively removed to simulate occlusion
    % fCx is the X coordinate of each triangle's centroid
    fCx    = (vertices(faces(:,1),1) + vertices(faces(:,2),1) + vertices(faces(:,3),1)) / 3;
    occIdx = find(fCx > mean(vertices(:,1)));  % indices of triangles on the +X side

    regionCounts_occ = zeros(nOcc,1);   % store region count per occlusion level
    regionOK_occ     = cell(nOcc,1);    % store per-slice feasibility per occlusion level

    for oi = 1:nOcc
        % calculate how many triangles to remove at this occlusion level
        nRem = round(occlusionFractions(oi) * numel(occIdx));

        % build a logical mask — start with all triangles kept (true)
        % then set the first nRem occluded triangles to false (removed)
        keepMask = true(size(faces,1),1);
        keepMask(occIdx(1:nRem)) = false;
        faces_occ = faces(keepMask,:);  % reduced face list with occluded triangles removed

        fprintf('Occlusion %.0f%%: removed %d triangles\n', ...
            occlusionFractions(oi)*100, nRem);

        % slice the occluded mesh at every height
        allLoops_occ = cell(n,1);
        for i = 1:n
            segs = sliceMesh(vertices, faces_occ, zValues(i));
            allLoops_occ{i} = stitchSegments(segs);
        end

        % diameter check with deformability correction
        % this mirrors the main analyzeGrasp logic so results are comparable
        sliceOK_occ = false(n,1);
        for i = 1:n
            lps = allLoops_occ{i};
            if isempty(lps), continue; end

            % pick largest loop at this height
            [~,li] = max(cellfun(@(L) size(L,1), lps));
            lp = lps{li};

            % skip contours with too few points to be meaningful
            if size(lp,1) < 5, continue; end

            % compute raw rigid-body diameter using bounding box
            D = max(max(lp(:,1))-min(lp(:,1)), max(lp(:,2))-min(lp(:,2)));

            % apply deformability correction — same factor as main analysis
            % so that occlusion results reflect the same effective diameter
            % the gripper would actually encounter on a real soft fruit
            D_eff = D * (1 - cfg.deformFactor);

            % check if effective diameter fits the gripper span
            sliceOK_occ(i) = (D_eff >= cfg.minGripSpan) && (D_eff <= cfg.maxGripSpan);
        end

        % count valid finger slots at this occlusion level
        % same logic as analyzeGrasp: consecutive feasible slices must span
        % at least fingerWidth3D in height to count as a valid region
        nReg=0; inReg=false; rStart=0;
        for i = 1:n
            if sliceOK_occ(i) && ~inReg
                inReg  = true;
                rStart = i;
            elseif ~sliceOK_occ(i) && inReg
                if (i-rStart)*delta >= cfg.fingerWidth3D
                    nReg = nReg+1;
                end
                inReg = false;
            end
        end
        % handle region running all the way to the last slice
        if inReg && (n-rStart+1)*delta >= cfg.fingerWidth3D
            nReg = nReg+1;
        end

        regionCounts_occ(oi) = nReg;
        regionOK_occ{oi}     = sliceOK_occ;
        fprintf('  → %d valid region(s)\n', nReg);
    end
end

% -------------------------------------------------------------------------
function runValidation(vertices, faces, zValues, allLoops, sliceDia, sliceDia_rigid, cfg, delta, closingAxis)
% Geometric validation: Test A (resolution convergence) and Test B (diameter accuracy)
%
% sliceDia       — effective diameter after deformability correction (used in main analysis)
% sliceDia_rigid — raw rigid-body diameter before correction (used in Test B verification)
% closingAxis    — 1x2 unit vector of gripper closing direction (from pass 2),
%                  or empty [] if no valid regions were found in pass 1
%
% Test B compares against sliceDia_rigid because the convex hull recomputes
% the raw geometric diameter — comparing against sliceDia would always show
% a discrepancy equal to the deformability factor, which is expected not an error.
% Test B also uses the same diameter method (oriented or global) as the main
% analysis so the comparison is meaningful.

    fprintf('\nNow introducing section 7: validation\n');
    n = numel(zValues);

    % check if an oriented closing axis was used in the main analysis
    % nargin counts how many arguments were passed in — 9 means closingAxis was provided
    useOriented = (nargin == 9) && ~isempty(closingAxis);

    % --- Test A: Resolution Convergence ---
    fprintf('\nBelow is Test A: Resolution Convergence\n');
    nSlice_tests = [10, 20, 30, 50];
    feasStart    = zeros(size(nSlice_tests));
    feasEnd      = zeros(size(nSlice_tests));

    for ti = 1:numel(nSlice_tests)
        ns   = nSlice_tests(ti);
        zv   = linspace(min(vertices(:,3)), max(vertices(:,3)), ns);
        dias = nan(ns,1);

        for i = 1:ns
            segs = sliceMesh(vertices, faces, zv(i));
            if isempty(segs), continue; end
            dias(i) = max(max(segs(:,1))-min(segs(:,1)), max(segs(:,2))-min(segs(:,2)));
        end

        ok = (dias >= cfg.minGripSpan) & (dias <= cfg.maxGripSpan);
        if any(ok)
            feasStart(ti) = zv(find(ok,1,'first'));
            feasEnd(ti)   = zv(find(ok,1,'last'));
        end
        fprintf('  %2d slices: Z=[%.4f, %.4f] m\n', ns, feasStart(ti), feasEnd(ti));
    end

    % --- Test B: Diameter Verification ---
    fprintf('\nBelopw is Test B: Diameter Verification\n');
    nErr = 0; nChk = 0; tol_chk = 1e-4;

    for i = 1:n
        if isnan(sliceDia_rigid(i)), continue; end
        lps = allLoops{i};
        if isempty(lps), continue; end
        [~,li] = max(cellfun(@(L) size(L,1), lps));
        lp = lps{li};

        try
            k        = convhull(lp(:,1), lp(:,2));
            hull_pts = lp(k, 1:2);

            if useOriented
                D_gt = computeOrientedDiameter(hull_pts, closingAxis);
            else
                D_gt = 0;
                for pp1 = 1:size(hull_pts,1)
                    for pp2 = pp1+1:size(hull_pts,1)
                        d = norm(hull_pts(pp1,:) - hull_pts(pp2,:));
                        if d > D_gt, D_gt = d; end
                    end
                end
            end
        catch
            D_gt = max(max(lp(:,1))-min(lp(:,1)), max(lp(:,2))-min(lp(:,2)));
        end

        nChk = nChk + 1;
        if abs(D_gt - sliceDia_rigid(i)) > tol_chk
            nErr = nErr + 1;
        end
    end

    if nErr == 0
        fprintf('   All %d values verified, there are no discrepancies above %.1f mm\n', nChk, tol_chk*1000);
    else
        fprintf('   %d/%d values had discrepancies > %.1f mm\n', nErr, nChk, tol_chk*1000);
    end

    % --- Validation Figure ---
    figure('Name','Validation','Position',[100 100 900 500]);

    subplot(1,2,1);
    errorbar(nSlice_tests, (feasStart+feasEnd)/2, (feasEnd-feasStart)/2, '-o', 'LineWidth', 2);
    xlabel('Num Slices'); ylabel('Feasible Z midpoint (m)');
    title('Test A: Resolution Convergence'); grid on; xticks(nSlice_tests);

    subplot(1,2,2);
    diffs = nan(n,1);
    for i = 1:n
        if isnan(sliceDia_rigid(i)), continue; end
        lps = allLoops{i};
        if isempty(lps), continue; end
        [~,li] = max(cellfun(@(L) size(L,1), lps));
        lp = lps{li};

        try
            k        = convhull(lp(:,1), lp(:,2));
            hull_pts = lp(k, 1:2);

            if useOriented
                D_recomputed = computeOrientedDiameter(hull_pts, closingAxis);
            else
                D_recomputed = max(max(lp(:,1))-min(lp(:,1)), max(lp(:,2))-min(lp(:,2)));
            end
        catch
            D_recomputed = max(max(lp(:,1))-min(lp(:,1)), max(lp(:,2))-min(lp(:,2)));
        end

        diffs(i) = abs(D_recomputed - sliceDia_rigid(i));
    end

    bar(zValues, diffs*1000, 'FaceColor',[0.2 0.7 0.4], 'EdgeColor','none');
    yline(tol_chk*1000, '--r', 'Threshold', 'LineWidth', 1.5);
    xlabel('Z (m)'); ylabel('Error (mm)');
    title('Test B: Diameter Accuracy'); grid on;

    sgtitle('Section 7: Geometric Validation');

end
% -------------------------------------------------------------------------
function exportGraspCSV(S9, cfg)
% Write grasp pose to CSV for Gazebo.
% Columns: fruit, gripper, grasp_x, grasp_y, grasp_z,
%          approach_dx, approach_dy, approach_dz, gap_angle_deg
    if isempty(S9)
        fprintf('No grasp found, CSV not written.\n');
        return;
    end
    fid = fopen(cfg.csvFile, 'w');
    fprintf(fid, 'fruit,gripper,grasp_x,grasp_y,grasp_z,approach_dx,approach_dy,approach_dz,gap_angle_deg\n');
    fprintf(fid, '%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f\n', ...
        cfg.fruitName, cfg.gripperName, ...
        S9.centroid(1), S9.centroid(2), S9.centroid(3), ...
        S9.approachDir(1), S9.approachDir(2), S9.approachDir(3), ...
        S9.gapAngleDeg);
    fclose(fid);
    fprintf('Grasp data written to %s\n', cfg.csvFile);
end
% -------------------------------------------------------------------------
function plotDiameterProfile(zValues, sliceDia, sliceOK, fingerSlots, cfg)
    plot(zValues, sliceDia, '-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
    yline(cfg.minGripSpan, '--r', 'Min Grip', 'LineWidth', 2);
    yline(cfg.maxGripSpan, '--g', 'Max Grip', 'LineWidth', 2);
    feasZ = zValues(sliceOK);  feasD = sliceDia(sliceOK);
    if ~isempty(feasZ)
        scatter(feasZ, feasD, 80, 'filled', 'MarkerFaceColor', 'm');
    end
    maxD = max(sliceDia(~isnan(sliceDia)));
    for r = 1:size(fingerSlots,1)
        fill([zValues(fingerSlots(r,1)), zValues(fingerSlots(r,2)), ...
              zValues(fingerSlots(r,2)), zValues(fingerSlots(r,1))], ...
             [0 0 maxD*1.1 maxD*1.1], 'b', 'FaceAlpha',0.15,'EdgeColor','b');
    end
    xlabel('Height Z (m)'); ylabel('Diameter (m)');
    title('Cross-Section Diameter vs Height'); grid on; hold off;
end

% -------------------------------------------------------------------------
function plotMug3D(allLoops, fingerSlots, numSlices)
    hold on; axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    for i=1:numSlices
        for j=1:numel(allLoops{i})
            lp=allLoops{i}{j};
            plot3(lp(:,1),lp(:,2),lp(:,3),'-','Color',[0.75 0.75 0.75],'LineWidth',0.5);
        end
    end
    for r=1:size(fingerSlots,1)
        for i=fingerSlots(r,1):fingerSlots(r,2)
            for j=1:numel(allLoops{i})
                lp=allLoops{i}{j};
                plot3(lp(:,1),lp(:,2),lp(:,3),'-r','LineWidth',3);
            end
        end
    end
    view(3);
end

% -------------------------------------------------------------------------
function printGraspResults(fingerSlots, zValues, delta, cfg)
    fprintf('\nIntroducing power grasp results\n');
    fprintf('Hand: grip span [%.1f, %.1f] mm | finger 3D width %.1f mm\n', ...
        cfg.minGripSpan*1000, cfg.maxGripSpan*1000, cfg.fingerWidth3D*1000);
    if isempty(fingerSlots)
        fprintf('No graspable regions found.\n\n');
        return;
    end
    fprintf('%d graspable region(s):\n\n', size(fingerSlots,1));
    for r=1:size(fingerSlots,1)
        zs=zValues(fingerSlots(r,1)); ze=zValues(fingerSlots(r,2));
        h=ze-zs;
        fprintf('  Region %d: Z=[%.4f, %.4f] m  height=%.4f m  ~%d finger(s)\n', ...
            r, zs, ze, h, floor(h/cfg.fingerWidth3D));
    end
    maxH = max(fingerSlots(:,2)-fingerSlots(:,1)+1)*delta;
    nF   = floor(maxH/cfg.fingerWidth3D);
    if nF>=3,     fprintf('strong power grasp (3+ fingers)\n');
    elseif nF>=2, fprintf('stable power grasp (2 fingers)\n');
    else,         fprintf('weak grasp (1 finger)\n');
    end
    fprintf('\n');
end

% -------------------------------------------------------------------------
function [optMin, optMax, sweepTable] = gripperSpanSweep(sliceDia, sliceDia_rigid, zValues, delta, cfg)
% gripperSpanSweep — sweeps gripper min/max span combinations and finds
% the optimal span range for this fruit.
%
% Uses sliceDia (effective diameter after deformation correction) so that
% gripper span recommendations are consistent with the main feasibility
% analysis. sliceDia_rigid is stored in the table for reference only.
%
% INPUTS:
%   sliceDia       — Nx1 effective diameters after deformFactor correction
%   sliceDia_rigid — Nx1 raw rigid-body diameters before correction
%   zValues        — Nx1 slice heights in meters
%   delta          — slice spacing in meters
%   cfg            — config struct (needs cfg.fingerWidth3D, cfg.fruitName)
%
% OUTPUTS:
%   optMin     — optimal minimum gripper span (m)
%   optMax     — optimal maximum gripper span (m)
%   sweepTable — struct containing full sweep results for plotting

    % define sweep ranges
    minSpans = 0.010:0.005:0.060;   % 10mm to 60mm in 5mm steps
    maxSpans = 0.040:0.005:0.120;   % 40mm to 120mm in 5mm steps

    nMin = numel(minSpans);
    nMax = numel(maxSpans);
    n    = numel(zValues);

    regionCounts  = zeros(nMin, nMax);  % number of valid finger slots per combo
    regionHeights = zeros(nMin, nMax);  % height of tallest slot per combo

    for mi = 1:nMin
        for mxi = 1:nMax
            mnS = minSpans(mi);
            mxS = maxSpans(mxi);

            % skip combinations where min >= max (invalid gripper)
            if mnS >= mxS, continue; end

            % skip combinations where span range is too narrow to be useful
            % a gripper that can only open 20mm total has no practical range
            if (mxS - mnS) < 0.020, continue; end

            % check each slice against this gripper combo
            % uses sliceDia (effective) not sliceDia_rigid so recommendations
            % are consistent with the deformability-corrected main analysis
            ok = (sliceDia >= mnS) & (sliceDia <= mxS) & ~isnan(sliceDia);

            % count valid finger slots for this gripper combo
            nReg=0; inReg=false; rStart=0; maxH=0;
            for i=1:n
                if ok(i) && ~inReg
                    inReg  = true;
                    rStart = i;
                elseif ~ok(i) && inReg
                    h = (i-rStart)*delta;
                    if h >= cfg.fingerWidth3D
                        nReg = nReg+1;
                        if h > maxH, maxH = h; end
                    end
                    inReg = false;
                end
            end
            % handle region running to last slice
            if inReg
                h = (n-rStart+1)*delta;
                if h >= cfg.fingerWidth3D
                    nReg = nReg+1;
                    if h > maxH, maxH = h; end
                end
            end

            regionCounts(mi,mxi)  = nReg;
            regionHeights(mi,mxi) = maxH;
        end
    end

    % find optimal combo: most regions first, then narrowest span
    % narrowest span = most specific gripper recommendation
    [maxReg, ~] = max(regionCounts(:));
    candidates  = find(regionCounts == maxReg);

    spanWidths = zeros(size(candidates));
    for ci = 1:numel(candidates)
        [mi, mxi]      = ind2sub([nMin, nMax], candidates(ci));
        spanWidths(ci) = maxSpans(mxi) - minSpans(mi);
    end
    [~, bestC]        = min(spanWidths);
    bestIdx           = candidates(bestC);
    [bestMi, bestMxi] = ind2sub([nMin, nMax], bestIdx);

    optMin = minSpans(bestMi);
    optMax = maxSpans(bestMxi);

    % store everything in sweepTable for plotting and reporting
    sweepTable.minSpans      = minSpans;
    sweepTable.maxSpans      = maxSpans;
    sweepTable.regionCounts  = regionCounts;
    sweepTable.regionHeights = regionHeights;

    % print summary
    fprintf('\nIntroducing gripper span sweep: %s\n', cfg.fruitName);
    fprintf('Deformability factor applied: %.0f%%\n', cfg.deformFactor*100);
    fprintf('Best region count: %d\n', maxReg);
    fprintf('Optimal span: [%.1f, %.1f] mm (effective diameters)\n', optMin*1000, optMax*1000);
    fprintf('Grasp height: %.1f mm\n', regionHeights(bestMi,bestMxi)*1000);
end

% -------------------------------------------------------------------------
function D = computeOrientedDiameter(hull_pts, closingAxis)
% computeOrientedDiameter — measures the width of a convex hull
% specifically along a given direction (the gripper's closing axis).
%
% hull_pts:    Px2 matrix of XY coordinates of convex hull points
% closingAxis: 1x2 unit vector [cx cy] — the direction gripper fingers close
%
% Returns D: the projected width of the shape along closingAxis
%
% How it works:
% Each hull point is projected onto the closing axis using a dot product.
% The projection scalar tells you how far along the axis that point sits.
% The width is the difference between the furthest and nearest projections.

    projections = hull_pts * closingAxis(:);
    % hull_pts is Px2 = [x1 y1; x2 y2; ...]
    % closingAxis(:) converts [cx cy] to a column vector [cx; cy]
    % matrix multiply: each row [xi yi] dot [cx; cy] = xi*cx + yi*cy
    % result is a Px1 vector of scalar projections

    D = max(projections) - min(projections);
    % width = furthest projection minus nearest projection
end
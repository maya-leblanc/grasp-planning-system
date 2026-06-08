%% HELPER FUNCTIONS FOR MULTI-OBJECT FRUIT PICKING GRASP PLANNER
% These functions are extracted from your Sections 1-12 and simplified
% for use in the multi-object loop.

% =========================================================================
% FUNCTION 1: Load and mesh an STL file
% =========================================================================
function [vertices, faces] = load_and_mesh_stl(stl_file)
    % Load STL and generate mesh
    model = createpde();
    importGeometry(model, stl_file);
    meshData = generateMesh(model, 'Hmax', 2);
    
    vertices = meshData.Nodes';  % Nx3
    faces = meshData.Elements';  % Mx3
    
    % Unit conversion: if vertices are in mm, convert to meters
    if max(abs(vertices(:))) > 10
        vertices = vertices / 1000;
    end
    
    fprintf('    Loaded %s: %d vertices, %d faces\n', stl_file, size(vertices,1), size(faces,1));
end

% =========================================================================
% FUNCTION 2: Multi-height slicing of the mesh
% =========================================================================
function [allLoops, zValues, numSlices] = slice_mesh_multiheight(vertices, faces)
    % Slice the mesh at multiple heights and extract contour loops
    
    numSlices = 20;
    z_min = min(vertices(:,3));
    z_max = max(vertices(:,3));
    zValues = linspace(z_min, z_max, numSlices);
    
    allLoops = cell(numSlices, 1);
    tolerance = 1e-6;
    
    for i = 1:numSlices
        z0 = zValues(i);
        
        % Find triangle-plane intersections (your Section 2 logic)
        segments = [];
        for f = 1:size(faces, 1)
            idx = faces(f, :);
            triVerts = vertices(idx, :);
            zVals = triVerts(:, 3);
            
            if (max(zVals) >= z0) && (min(zVals) <= z0) && ~all(zVals == z0)
                pts = [];
                
                for e = [1 2; 2 3; 3 1]'
                    p1 = triVerts(e(1), :);
                    p2 = triVerts(e(2), :);
                    
                    z1 = p1(3) - z0;
                    z2 = p2(3) - z0;
                    
                    if z1 * z2 < 0
                        t = z1 / (z1 - z2);
                        intersectPt = p1 + t * (p2 - p1);
                        pts = [pts; intersectPt];
                    elseif z1 == 0 && z2 ~= 0
                        pts = [pts; p1];
                    elseif z2 == 0 && z1 ~= 0
                        pts = [pts; p2];
                    end
                end
                
                if size(pts, 1) == 2
                    segments = [segments; pts];
                end
            end
        end
        
        % Stitch segments into loops (your Section 3 logic)
        loops = {};
        while size(segments, 1) >= 2
            pt1 = segments(1, :);
            pt2 = segments(2, :);
            segments(1:2, :) = [];
            
            loop = [pt1; pt2];
            loopClosed = false;
            
            while ~loopClosed
                found = false;
                
                for j = 1:2:size(segments, 1)-1
                    seg_pt1 = segments(j, :);
                    seg_pt2 = segments(j+1, :);
                    
                    if norm(seg_pt1 - loop(end, :)) < tolerance
                        loop = [loop; seg_pt2];
                        segments(j:j+1, :) = [];
                        found = true;
                        break
                    elseif norm(seg_pt2 - loop(end, :)) < tolerance
                        loop = [loop; seg_pt1];
                        segments(j:j+1, :) = [];
                        found = true;
                        break
                    elseif norm(seg_pt1 - loop(1, :)) < tolerance
                        loop = [seg_pt2; loop];
                        segments(j:j+1, :) = [];
                        found = true;
                        break
                    elseif norm(seg_pt2 - loop(1, :)) < tolerance
                        loop = [seg_pt1; loop];
                        segments(j:j+1, :) = [];
                        found = true;
                        break
                    end
                end
                
                if ~found
                    loopClosed = true;
                end
            end
            
            loops{end+1} = loop;
        end
        
        allLoops{i} = loops;
    end
    
end

% =========================================================================
% FUNCTION 3: Power grasp feasibility analysis (your Section 8 logic)
% =========================================================================
function [graspable_slices, sliceDia, fingerSlots] = ...
    analyze_power_grasp(allLoops, zValues, numSlices, params)
    
    delta = mean(diff(zValues));
    sliceOK = false(numSlices, 1);
    sliceDia = nan(numSlices, 1);
    
    for i = 1:numSlices
        loops = allLoops{i};
        if isempty(loops)
            continue;
        end
        
        % Project loops to XY and compute diameter
        polys = cellfun(@(L)[L(:,1:2); L(1,1:2)], loops, 'UniformOutput', false);
        nPoly = numel(polys);
        
        % Compute area, perimeter, diameter for each polygon
        APD = zeros(nPoly, 3);
        for k = 1:nPoly
            poly = polys{k};
            x = poly(:, 1); y = poly(:, 2);
            A = 0.5 * abs(sum(x(1:end-1).*y(2:end) - x(2:end).*y(1:end-1)));
            P = sum(sqrt(sum(diff(poly, 1, 1).^2, 2)));
            rA = sqrt(A/pi);
            rP = P/(2*pi);
            D = rA + rP;
            APD(k, :) = [A, P, D];
        end
        
        % Determine feasibility based on number of polygons
        if nPoly == 1
            D = APD(1, 3);
            sliceDia(i) = D;
            if D >= params.minGripSpan && D <= params.maxGripSpan
                sliceOK(i) = true;
            end
        elseif nPoly == 2
            % Two polygons: check gap
            polyA = polys{1};
            polyB = polys{2};
            
            minGap = compute_polygon_gap(polyA, polyB);
            [~, idxLarge] = max(APD(:, 1));
            D = APD(idxLarge, 3);
            sliceDia(i) = D;
            
            if minGap >= params.fingerWidth2D && ...
               D >= params.minGripSpan && D <= params.maxGripSpan
                sliceOK(i) = true;
            end
        else
            % Multiple polygons
            [~, idxLarge] = max(APD(:, 1));
            D = APD(idxLarge, 3);
            sliceDia(i) = D;
            
            bestGap = 0;
            polyLarge = polys{idxLarge};
            for k = 1:nPoly
                if k == idxLarge, continue; end
                polyOther = polys{k};
                bestGap = max(bestGap, compute_polygon_gap(polyLarge, polyOther));
            end
            
            if bestGap >= params.fingerWidth2D && ...
               D >= params.minGripSpan && D <= params.maxGripSpan
                sliceOK(i) = true;
            end
        end
    end
    
    % Accumulate consecutive slices into 3D finger regions
    fingerSlots = [];
    inRegion = false;
    regionStart = 0;
    
    for i = 1:numSlices
        if sliceOK(i) && ~inRegion
            inRegion = true;
            regionStart = i;
        elseif ~sliceOK(i) && inRegion
            regionEnd = i - 1;
            regionHeight = (regionEnd - regionStart + 1) * delta;
            
            if regionHeight >= params.fingerWidth3D
                fingerSlots = [fingerSlots; regionStart, regionEnd];
            end
            inRegion = false;
        end
    end
    
    if inRegion
        regionEnd = numSlices;
        regionHeight = (regionEnd - regionStart + 1) * delta;
        if regionHeight >= params.fingerWidth3D
            fingerSlots = [fingerSlots; regionStart, regionEnd];
        end
    end
    
    graspable_slices = find(sliceOK);
end

% =========================================================================
% HELPER: Compute minimum gap between two polygons
% =========================================================================
function minGap = compute_polygon_gap(polyA, polyB)
    minGap = inf;
    
    for ia = 1:size(polyA, 1)-1
        a1 = polyA(ia, :);
        a2 = polyA(ia+1, :);
        
        for jb = 1:size(polyB, 1)-1
            b1 = polyB(jb, :);
            b2 = polyB(jb+1, :);
            
            % Point-to-segment distances (4 combinations)
            pts = {b1,a1,a2; b2,a1,a2; a1,b1,b2; a2,b1,b2};
            for p = 1:size(pts, 1)
                pVec = pts{p, 1};
                aVec = pts{p, 2};
                bVec = pts{p, 3};
                ab = bVec - aVec;
                t = max(0, min(1, dot(pVec-aVec, ab)/max(dot(ab,ab), eps)));
                proj = aVec + t*ab;
                minGap = min(minGap, norm(pVec - proj));
            end
        end
    end
end

% =========================================================================
% FUNCTION 4: Compute approach direction (your Section 9 logic)
% =========================================================================
function S9 = compute_approach_direction(allLoops, zValues, fingerSlots, sliceDia)
    
    S9 = struct();
    
    if isempty(fingerSlots)
        return;
    end
    
    % Pick best (tallest) region
    regionHeights = (fingerSlots(:, 2) - fingerSlots(:, 1) + 1) * mean(diff(zValues));
    [~, bestR] = max(regionHeights);
    bestStart = fingerSlots(bestR, 1);
    bestEnd = fingerSlots(bestR, 2);
    
    % Midpoint slice
    midSliceIdx = round((bestStart + bestEnd) / 2);
    midZ = zValues(midSliceIdx);
    
    % Get largest loop at midpoint
    midLoops = allLoops{midSliceIdx};
    if isempty(midLoops)
        for offset = 1:3
            if midSliceIdx+offset <= length(allLoops) && ~isempty(allLoops{midSliceIdx+offset})
                midLoops = allLoops{midSliceIdx+offset};
                break;
            end
        end
    end
    
    if isempty(midLoops)
        return;
    end
    
    loopLengths = cellfun(@(L) size(L, 1), midLoops);
    [~, lgIdx] = max(loopLengths);
    midLoop = midLoops{lgIdx};
    
    % Centroid
    cx = mean(midLoop(:, 1));
    cy = mean(midLoop(:, 2));
    cz = midZ;
    centroid = [cx, cy, cz];
    
    % Find approach direction via widest angular gap
    angles = atan2(midLoop(:, 2) - cy, midLoop(:, 1) - cx);
    angles_sorted = sort(angles);
    
    gaps = diff(angles_sorted);
    wrap_gap = (2*pi - angles_sorted(end)) + angles_sorted(1);
    all_gaps = [gaps; wrap_gap];
    
    [maxGap, gapIdx] = max(all_gaps);
    if gapIdx < length(all_gaps)
        gapMidAngle = angles_sorted(gapIdx) + all_gaps(gapIdx)/2;
    else
        gapMidAngle = angles_sorted(end) + wrap_gap/2;
        if gapMidAngle > pi, gapMidAngle = gapMidAngle - 2*pi; end
    end
    
    % Approach direction (inward)
    approachDir = [-cos(gapMidAngle), -sin(gapMidAngle), 0];
    approachDir = approachDir / norm(approachDir);
    
    % Store results
    S9.bestStart = bestStart;
    S9.bestEnd = bestEnd;
    S9.midSliceIdx = midSliceIdx;
    S9.centroid = centroid;
    S9.approachDir = approachDir;
    S9.gapAngleDeg = rad2deg(gapMidAngle);
    
end

% =========================================================================
% FUNCTION 5: Compute grasp quality metrics
% =========================================================================
function [epsilon_metric, Q1_metric, force_closure_passes] = ...
    compute_grasp_quality(S9, allLoops, zValues, params)
    
    epsilon_metric = 0;
    Q1_metric = 0;
    force_closure_passes = false;
    
    if ~isfield(S9, 'centroid') || isempty(S9.centroid)
        return;
    end
    
    % Extract contact points from the midpoint loop
    midLoops = allLoops{S9.midSliceIdx};
    if isempty(midLoops)
        return;
    end
    
    loopLengths = cellfun(@(L) size(L, 1), midLoops);
    [~, lgIdx] = max(loopLengths);
    midLoop = midLoops{lgIdx};
    
    % Simplified: estimate epsilon as the minimum distance from centroid
    % to the contour (robustness margin)
    centroid_xy = S9.centroid(1:2);
    distances_to_contour = vecnorm((midLoop(:,1:2) - centroid_xy)')';
    epsilon_metric = min(distances_to_contour);
    
    % Q1 metric: approximate as the ratio of smallest to largest principal axis
    % (for now, simplified as epsilon / max_distance)
    max_dist = max(distances_to_contour);
    Q1_metric = epsilon_metric / max(max_dist, 1e-6);
    
    % Force closure: check if epsilon is positive (grasp is feasible)
    force_closure_passes = (epsilon_metric > 0.5*params.minGripSpan);
    
end
% Authour: Maya LeBlanc
% Affiliation: McMaster University
% Start Date: August 12 2025
% Referal: Dr. Gary Bone, Department of Mechanical Engineering, Robotics
% and Manufacturing Automation Laboratory (RMAL), McMaster University
% Type: Grasp Planning Engineering Computing Challenge


% In this grasp planning programming challenge, referred from Dr. Gary Bone, the goal is
% to hyperanalyze an stl file of a mug in order to determine if a 2-3 joint
% or 3 fingered robotic hand is able to hold a "power grasp" or an
% "enveloping grasp". 

% The first and main problem of the entire grasp planning problem is to find the regions on the mug where a power grasp
% can be held by a robotic hand. In order to find this, there are many
% stages that I do not yet know. However, as hinted by Dr. Bone, we need to
% turn the stl file of the mug into a mesh, and then create a
% cross-sectional view of the relationships between the z plane (thats how
% you make a cross-section) and the mug, through slicing. 

% All I aim to do now is to try and figure out how to do these two things
% and fully understanding them before moving on to anything else. I have
% finished transforming the mug to a mesh polygon, and now I must figure
% out how to create a cross-sectional view hyperanalysing everything. 




%% Section 1: Creating a Mesh.

% Step 1: Create a PDE model and load geometry. In order to turn an stl
% file into a mesh polygon, it is different that just clicking a button and
% turning it into a mesh. Abstractly, you must create an empty box, load
% the stl file you want a mesh of into it, and then press the generate a
% mesh button. 

% The empty box is called a generic PDE model. This object is a containter
% that stores information about the partial differential equation (PDE)
% problem you want to solve, including the geometry, mesh, and boundary
% conditions. It essentialy sets up the framework for solving a PDE using
% the PDE toolbox. 

model = createpde();             % Create a generic PDE model
importGeometry(model,"mug.stl"); % Load the STL into the model. 
% Import the geometry (you define if stl file or not in purple); model
% means the variable you are refering to when creating the PDE model (which
% we have created a line above); and the stl file must be in the same file
% on the computer where you have saved this MATLAB code, so MATLAB can find
% it. Must name it model.

% Step 2: Generate a mesh for that geometry
meshData = generateMesh(model,'Hmax',2);  % Smaller Hmax → finer mesh
% What does 2 mean...? or Hmax...?

% Step 3:  Extract vertices and faces
vertices = meshData.Nodes';     % Nx3 (X,Y,Z)
faces    = meshData.Elements';  % Mx3 (vertex indices)

% We need to find the vertices and faces and visualize them. Create new
% variables per vertices and faces and take the meshData variable (random
% name involved) and from the mesh we have created, transform into vertices
% and faces per variable. .Nodes' for vertices means... and .Elements' for
% faces means... In each variable, there will be a value per face/vertice
% in a matrix array, each #x3. 

% % Step 4: Visualize to check
% figure
% pdemesh(model)
% view(3)

% figure asks for making a figure on MATLAB in order to generate mesh
% pdemesh(model) plots the mesh contained in a 2-D or 3-D model object.
% View model in 3D


% Super helpful link for this section: https://www.mathworks.com/help/pde/ug/geometry-and-mesh-components.html


minVals = min(vertices);  % [minX, minY, minZ]
maxVals = max(vertices);  % [maxX, maxY, maxZ]
sizeVals = maxVals - minVals

% if vertices > 1000:
%     vertices = vertices / 1000;  % mm → m

% LOL FIX HEHE




%% Section 2: Z-Plane Intersects Mug and Slices for Cross-sectional Analysis

% In this section, ...


% We need to pick a slicing direction using a plane from the xyz plane. We
% usually slice horizontally along the z plane (like cutting a cake layer
% by layer)

z_min = min(vertices(:,3)) 
z_max = max(vertices(:,3))
% Save these two variables, as these values are your slicing range to
% choose a value inbetween for your z plane height. This value will be
% somewhere between the top and bottom of the mug, as these values have the
% max tall and max short coordinates.


z0 = 0.05;  % 5 cm above base (we have chosen this based on the slicing range from above) (and we are assuming meters)

% Now we shall implement a single slice. : 
% 
% We want to "slice" a 3D shape (made of triangles) with a flat plane (say, z = z0).
% That slice is not a solid — it's a curve (a loop of line segments) that shows where the plane cuts through the shape.
% So, to get that slice, we need to:
%
% Look at every triangle in the mesh.
% Ask: does this triangle get cut by the plane?
% If yes: find the exact 2D line segment where the plane intersects that triangle.
% Collect all those little line segments → together they form the outline of the mug at that height.


% Step 1: Test Slice Height - we have as z0 = 0.05 m


% Step 2: Loop Over Each Triangle. Packed with Geometry Logic. The actual
% description of how to slice is now here at its heart. 

segments = []; % to store intersection line segments

for i = 1:size(faces,1) % Get indices of the triangle's 3 vertices; Loops through every triangle in the mesh (faces is the list of triangles).
   
    idx = faces(i,:); 
    triVerts = vertices(idx,:); % Gets the 3 vertices of the current triangle.
    
    zVals = triVerts(:,3); % Extracts the z-coordinates of the 3 vertices → to check where they are compared to the slicing plane.
    
    if (max(zVals) >= z0) && (min(zVals) <= z0) && ~all(zVals == z0) % Quesion 1: Does this triangle intersect the plane?
        % If one vertex is above the plane (z > z0) and one below (z < z0), the plane must cut this triangle.
        % The extra ~all(zVals == z0) avoids the special case where the entire triangle lies flat on the slicing plane.
        
        pts = []; % Store points where edges cross the plane
        
        % Check each edge of the triangle
        for e = [1 2; 2 3; 3 1]'  % Question 2: Find intersection points with edges; Loops over the 3 edges of the triangle: (v1-v2, v2-v3, v3-v1).
            p1 = triVerts(e(1),:);
            p2 = triVerts(e(2),:); % Gets the endpoints of this edge.
            
            z1 = p1(3) - z0;
            z2 = p2(3) - z0; % Measures whether each endpoint is above (+), below (-), or on (0) the plane.
            
            if z1 * z2 < 0  % one above, one below
                t = z1 / (z1 - z2); % interpolation fraction
                intersectPt = p1 + t * (p2 - p1); % If one point is above and the other below (signs differ), the edge must cross the plane. 
                % t is a fraction (linear interpolation) to find exactly where the edge intersects the plane.

                pts = [pts; intersectPt];
            elseif z1 == 0 && z2 ~= 0
                pts = [pts; p1]; % vertex exactly on plane
            elseif z2 == 0 && z1 ~= 0
                pts = [pts; p2]; % vertex exactly on plane; If a vertex lies exactly on the plane, we also record it.
            end
        end
        % Question 3: Save the Segment; A triangle can only be sliced into a straight line segment (two intersection points).
        % If we found exactly 2, we add that segment to our list.
        % If exactly two intersection points found → store as segment
        if size(pts,1) == 2
            segments = [segments; pts]; % append both points
        end
    end
end

% Why We Do All This
% Triangles are the smallest building blocks of the 3D model.
% Each triangle can intersect the slicing plane → which gives us one small line segment.
% By looping through all triangles, we gather every small piece of the slice.
% Later, we stitch those line segments together → that's the slice contour of the mug.
% So yes — we are "hyperanalyzing" the mug triangle by triangle, because that's the only 
% way to compute where a flat plane intersects a mesh. It's like running a laser scanner 
% across the mug, one triangle at a time.


% At this stage, we have the mesh of the mug, and A way to compute a single slice at height z0, 
% which gives you a bunch of line segments (pairs of 3D points)




%% Section 3: Assemble and Visualize the Slice Contour

% Right now, your segments array is just a pile of disconnected line segments.
% In order to connect them and make it an actual slice that works, we need
% to 1) organize and connect slice into a continuous loop; 2) Visualize; 3)
% Interpret

% Step 1: Organize the slice into a continuous loop
% Connect intersection points so that they form a closed contour (the cross-section outline of the mug at height z0).
% You currently have segments, which is a list of small line segments, each represented by two 3D points where 
% a triangle intersects the slicing plane. Right now, these are disconnected. You want a continuous loop 
% (or loops, if the mug has holes) representing the cross-section.
% In practice, each line segment has two endpoints. 
% You need to "stitch" them together by matching endpoints that are very close (within a tolerance).

% Tricky Hard Part but Fun: 


tolerance = 1e-6;
loops = {};

while size(segments, 1) >= 2
    % Take first segment pair
    pt1 = segments(1, :);
    pt2 = segments(2, :);
    segments(1:2, :) = [];
    
    loop = [pt1; pt2];
    loopClosed = false;
    
    while ~loopClosed
        found = false;
        
        % Try to extend the loop by finding connecting segments
        for j = 1:2:size(segments, 1)-1
            seg_pt1 = segments(j, :);
            seg_pt2 = segments(j+1, :);
            
            % Check all 4 connection possibilities
            if norm(seg_pt1 - loop(end, :)) < tolerance
                loop = [loop; seg_pt2];
                segments(j:j+1, :) = [];
                found = true;
                break;
            elseif norm(seg_pt2 - loop(end, :)) < tolerance
                loop = [loop; seg_pt1];
                segments(j:j+1, :) = [];
                found = true;
                break;
            elseif norm(seg_pt1 - loop(1, :)) < tolerance
                loop = [seg_pt2; loop];
                segments(j:j+1, :) = [];
                found = true;
                break;
            elseif norm(seg_pt2 - loop(1, :)) < tolerance
                loop = [seg_pt1; loop];
                segments(j:j+1, :) = [];
                found = true;
                break;
            end
        end
        
        if ~found
            loopClosed = true;
        end
    end
    
    loops{end+1} = loop;
end

% Step 2: Visualize

figure; hold on; axis equal; 
% Create a new figure window to plot in; 'hold on' allows multiple loops to be drawn on the same figure,
% 'axis equal' ensures X, Y, Z axes are scaled equally so the slice shape is not distorted.

for k = 1:length(loops) 
    % Loop over all loops stored in the cell array; each loop is a contour of the slice (could be outer boundary or hole).

    loop = loops{k}; 
    % Extract the k-th loop from the cell array; gives us the Nx3 array of points for this contour.

    plot3(loop(:,1), loop(:,2), loop(:,3), '-o', 'LineWidth', 2); 
    % Plot the 3D loop in space using X, Y, Z coordinates; '-o' draws points connected by lines,
    % 'LineWidth' makes the contour thicker for better visibility. This visualizes the cross-section.
end

xlabel('X'); ylabel('Y'); zlabel('Z'); 
% Label the axes for clarity; helps understand orientation of the slice in 3D space.

view(3); 
% Set the view to 3D perspective; allows rotating the plot and seeing the contour in 3D.


% Step 3: Interpret - done already above :D

% So far in your grasp-planning pipeline, you have successfully created a triangular mesh of the 
% mug by loading the STL file into a PDE model in MATLAB, generating the mesh, and extracting the 
% vertices and faces. You verified the mesh visually using pdemesh to ensure it accurately represents 
% the mug. Next, you defined a slicing plane at a chosen height z0 and looped through each triangle 
% in the mesh to determine whether it intersects the plane, calculating the exact intersection 
% points and storing them as line segments in the segments array. Using a loop-stitching algorithm, 
% you then connected these segments into continuous loops, where each loop represents a closed 
% contour of the mug's cross-section at that height. Finally, you visualized these loops in 3D with 
% plot3, labeled the axes, and set a 3D view, giving you a hyperanalyzed cross-sectional outline of 
% the mug. This process captures every triangle that intersects the slicing plane and sets the 
% foundation for analyzing potential grasp regions. The next steps are to automate slicing at multiple 
% heights to create a stack of cross-sections, analyze each slice for regions wide enough for a robotic 
% hand to perform a "power" or "enveloping" grasp, and eventually combine these slices into a full 
% 3D map of graspable regions.

% Right now we are have only one slice at a certain height (z0). Lets make
% multiple of them and combine them like a CT scan to fully analyze the
% mug/stl file, in order to find all potential grasp regions.




%% Section 4: Automating slicing across multiple heights

% Choose a slicing resolution (e.g., every 0.5 cm or 1 cm along z).
% For each slice height z0:
% Run the intersection loop (your triangle cutting code).
% Stitch the line segments into loops.
% Store the loops in a structured way (e.g., all_loops{sliceIndex} = loops).
% Visualize the stack of slices to see how the mug looks layer by layer (like a CT scan).
% This gives you the 2D contour at every height, which you'll then analyze to find where 
% the mug's geometry supports a power/enveloping grasp.

%% Section 4: Automating slicing across multiple heights

% Step 1: Define the range of z values to slice through
numSlices = 20;                                 
z_values = linspace(z_min, z_max, numSlices);  

% Step 2: Create a cell array to store slices
all_slices = cell(numSlices, 1);  

% Step 3: Loop through each z-value and compute slice contours
for i = 1:numSlices
    z0 = z_values(i);       
    lines = [];             
    
    % Step 3a: Loop through triangles to find intersections with plane z = z0
    for f = 1:size(faces,1)  % FIXED: was F
        v = vertices(faces(f,:), :);   % FIXED: was V and F
        z = v(:,3);         
        
        if (min(z) <= z0 && max(z) >= z0) && (max(z) ~= min(z))
            pts = [];
            for e = 1:3
                v1 = v(e,:);
                v2 = v(mod(e,3)+1,:);
                if ( (v1(3) - z0) * (v2(3) - z0) < 0 )
                    t = (z0 - v1(3)) / (v2(3) - v1(3));
                    p = v1 + t*(v2 - v1);
                    pts = [pts; p];
                elseif (v1(3) == z0)
                    pts = [pts; v1];
                end
            end
            if size(pts,1) == 2
                lines = [lines; pts];
            end
        end
    end
    
    all_slices{i} = lines;
end

% Step 4: Visualization of all slices
figure; hold on; axis equal;
title('Cross-sections of mug at different heights');
for i = 1:numSlices
    lines = all_slices{i};
    if ~isempty(lines)
        plot(lines(:,1), lines(:,2), '.-');
    end
end
xlabel('X'); ylabel('Y'); grid on;


%% Section 5: Analyze Slice Geometry for Graspability

% Now we have a stack of slices, we can start hyperanalyzing them.

% 1) Width measurement: For each slice loop, compute the maximum span across opposite points 
% (distance across the contour). This tells you how wide the mug is at that level.
% 2) Consistency check: A robotic power grasp requires not just width but also enough vertical 
% continuity (several consecutive slices with similar width) to ensure the hand encloses the mug.
% 3) Candidate grasp zones: Mark z-ranges where the mug is narrow enough for the robotic hand 
% but also wide enough to be stably gripped.

% This section will loop through many different slicing heights (z0 values), generate contours 
% for each slice, and then plot them together so you can see the "stacked" cross-sections of your mug.

% Define slicing heights from bottom to top of mug
numSlices = 20;                              % how many slices you want
zMin = min(vertices(:,3));                   % lowest point of mug
zMax = max(vertices(:,3));                   % highest point of mug
zLevels = linspace(zMin, zMax, numSlices);   % evenly spaced slicing heights

% Container to store all loops for every slice
allLoops = cell(numSlices,1);

% Loop through each z-level and compute cross-section loops
for i = 1:numSlices
    z0 = zLevels(i);   % current slice height
    
    % Re-use Section 3 (find triangle-plane intersections)
    segments = [];
    for f = 1:size(faces,1)
        tri = vertices(faces(f,:), :);   % triangle vertices
        zVals = tri(:,3);                % z-coordinates
        if (min(zVals) <= z0) && (max(zVals) >= z0)
            pts = [];
            for e = 1:3
                v1 = tri(e,:);
                v2 = tri(mod(e,3)+1,:);
                if (v1(3)-z0)*(v2(3)-z0) < 0
                    t = (z0-v1(3)) / (v2(3)-v1(3));
                    pts(end+1,:) = v1 + t*(v2-v1);
                elseif v1(3)==z0
                    pts(end+1,:) = v1;
                end
            end
            if size(pts,1)==2
                segments = [segments; pts];
            end
        end
    end
    
    % Re-use Section 4 (stitch segments into loops)
    loops = {};
    while ~isempty(segments)
        loop = segments(1:2,:);
        segments(1:2,:) = [];
        while true
            diffs = sqrt(sum((segments - loop(end,:)).^2,2));
            [minDist, idx] = min(diffs);
            if isempty(idx) || minDist > 1e-6
                break;
            end
            loop(end+1,:) = segments(idx,:);
            if size(loop,1) > 2 && norm(loop(end,:) - loop(1,:)) < 1e-6
                break;
            end
            segments(idx,:) = [];
        end
        loops{end+1} = loop;
    end
    
    % Store loops for this slice
    allLoops{i} = loops;
end

% Visualization of stacked slices
figure;
hold on;
colors = lines(numSlices);   % different colors for each slice
for i = 1:numSlices
    loops = allLoops{i};
    for j = 1:length(loops)
        loop = loops{j};
        plot3(loop(:,1), loop(:,2), loop(:,3), '-', 'Color', colors(i,:), 'LineWidth', 2);
    end
end
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Stacked Cross-Sections of the Mug');
view(3); grid on;


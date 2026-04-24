# grasp-planning-system

Power Grasp Detection on 3D Objects via Multi-Height Mesh Analysis

## Overview
This project, developed at the Robotics and Manufacturing Automation Laboratory (RMAL) at McMaster University, focuses on the automated detection of power-grasp regions for a robotic hand. By converting raw STL mesh data into a series of interconnected planar loops, the system can systematically evaluate where a physical gripper can safely "wrap" around an object without colliding with its own geometry.

## Problem: The Raw Input Challenge
- Geometrical Complexity. Raw STL files consist of thousands of underordered triangular faces. A robot cannot intuitively "know" which triangles constitute a handle versus the body.
- Physial Constraints. Robotic grippers have fixed mechanical limits.
  1. Minimum span, where the hand cannot close smaller than its physical structure
  2. Maxiumum span, where the hand cannot open wider than its joints allow
  3. Vertical footprint, where a stable grasp requires several centimeters of continuous contact, not just a single point
- The system must also detect "gaps" (e.g., the space between a mug handle and the body) to ensure a finger can actually fit through

## Solution: Slice-based geometric analysis
The algorithm employs a multi-stage pipeline to transform a 3D mesh into a Grasp Feasibility Map:

1. Spatial decomposition (slicing)
The code iterates through the Z-axis of the object, calculating the intersection of a plane at height z0​ with each triangular face.

- It finds the exact points where edges cross the plane using:

- These disconnected line segments are stiched into closed polygonal loops using a coordinate-matching algorithm with a 10^-6 tolerance

2. Topological analysis
Each slice is analysed for "graspability":
- 1 loop = a single wrap-round check
- 2+ loops = the algorithm calculates the minimum gap between polygons to determine if a robotic finger (of width params.fingerWidth2D) can pass through
- specific diameter = uses a combination of area (A) and perimeter (P) to estimate a "grasp diameter" (D) that is resilient to noisy mesh data:

3. Temporal (Z-axis) accumulation
Individual "OK" slices are not enough. The code looks for consecutive slices that pass the test, ensuring the object is tall enough to accomodate the physical width of a robotic finger (params.fingerWidth3D).

## Technical Stack
- MATLAB language
- Partial Differential Equation (PDE) Toolbox (for mesh processing and STL import)
- Algorithms include linear interpolation, polygon stiching, point-to-segment distance calculation, and geometric feature extraction
- Visualization includes 3D plotting (plot3), colormaps (jet), and subplot analysis for diameter-to-height ratios.

## Results
The script produces a detailed Power Grasp Analysis Report and visual diagnostics:
1. Quantitative Metrics:

- Identifies specifc z-ranges (in meters) where the object is graspable
- Calculates the capacity: determines how many fingers can fit in a specific region (e.g., "Region 1 can fit 3 fingers")
- Categorizes the grasp as Strong Power Grasp, Stable, or Weak/Marginal

2. Visual Diagnostics
- Diameter plot = a 2D graph showing where the object's width falls within the Red (Min) and Green (Max) grip limits
- A visual model where potential grasp zones are highlighed in bold red against a gray skeletal mesh

## Visuals
![Pipeline](images/pipeline.png)

## Full Code Access
The full implementation is maintained in a private repository.

**Full codebase available upon request for academic or professional review.**

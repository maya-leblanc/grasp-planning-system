function grippers = load_gripper_configs()
% Define your gripper geometries here
% Call this in your main script: grippers = load_gripper_configs();
    
grippers(1).name = 'Robotiq 3-Finger';
grippers(1).minGripSpan = 0.020;
grippers(1).maxGripSpan = 0.155;
grippers(1).urdf_file = 'robotiq_3f_gripper_articulated.urdf';

grippers(2).name = 'ContourGrip (Custom)';
grippers(2).minGripSpan = 0.005;
grippers(2).maxGripSpan = 0.015;
grippers(2).urdf_file = 'contourgripper.urdf';

end

end
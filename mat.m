%% Importing Robot and Setting an Initial Guess for IK
robotURDF = "C:\Users\Mayukh\Downloads\mycobot_pro600.urdf";
robot = importrobot(robotURDF);

% Setting the specified home configuration
homeConfig = homeConfiguration(robot);
homeConfig(1).JointPosition = deg2rad(46.484);    
homeConfig(2).JointPosition = deg2rad(-120.495);  
homeConfig(3).JointPosition = deg2rad(-107.298);     
homeConfig(4).JointPosition = deg2rad(-44.824);       
homeConfig(5).JointPosition = deg2rad(88.857);    
homeConfig(6).JointPosition = deg2rad(-8.877); 

ikInitGuess = homeConfig; % Initial guess is the custom home configuration

%% Read Multiple Coordinates from marker_coords.txt
markerData = readmatrix("C:\Users\Mayukh\Downloads\converted_coordinates.txt", 'Delimiter', ',', 'OutputType', 'char');

% Parse data into a matrix of 3D points
numPoints = size(markerData, 1); % Number of rows (points) in the file
points3D = zeros(numPoints, 3);

for i = 1:numPoints
    points3D(i, :) = str2double(split(markerData(i, :), ':'));
end

%% Setting up Inverse Kinematics Solver
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.PositionTolerance = 0;
ik.SolverParameters.OrientationTolerance = 0;
ik.SolverParameters.AllowRandomRestart = false;
weights = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5];
jointAngles = zeros(numPoints, 6);

%% Perform IK for Each Maze Coordinate with Visualization
orientation = eul2quat([175.846 * pi / 180, 0.863 * pi / 180, -85.628 * pi / 180], "XYZ");

figure;
for i = 1:numPoints
    targetPosition = points3D(i, :);
    pose = trvec2tform(targetPosition) * quat2tform(orientation);
    [configSoln, ~] = ik('Link_6', pose, weights, ikInitGuess);
    ikInitGuess = configSoln; % Update initial guess for the next point
    jointAngles(i, :) = [configSoln(1).JointPosition, ...
                         configSoln(2).JointPosition, ...
                         configSoln(3).JointPosition, ...
                         configSoln(4).JointPosition, ...
                         configSoln(5).JointPosition, ...
                         configSoln(6).JointPosition] * 180 / pi;

    % Append joint angles to CSV
    writematrix(jointAngles(i, :), 'angles.csv', 'WriteMode', 'append');

    % Visualization
    subplot(1, 2, 1);            
    show(robot, configSoln);     
    view([1, 0, 0]);             
    title('Front View');                                      
    subplot(1, 2, 2);
    show(robot, configSoln);
    view([0, 0, 1]);
    title('Top View');      
    sgtitle(sprintf('Step Number: %d', i));
    pause(1);
end

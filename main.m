%% Main function
clc
clear
close all

addpath('include')

%% Robot Model Definition
biTri(1,1,1) = 1; biTri(1,2,1) = 0; biTri(1,3,1) = 0; biTri(1,4,1) = 0;
biTri(2,1,1) = 0; biTri(2,2,1) = 1; biTri(2,3,1) = 0; biTri(2,4,1) = 0;
biTri(3,1,1) = 0; biTri(3,2,1) = 0; biTri(3,3,1) = 1; biTri(3,4,1) = 0.175;
biTri(4,1,1) = 0; biTri(4,2,1) = 0; biTri(4,3,1) = 0; biTri(4,4,1) = 1;

biTri(1,1,2) = -1; biTri(1,2,2) = 0;  biTri(1,3,2) = 0; biTri(1,4,2) = 0;
biTri(2,1,2) = 0; biTri(2,2,2) = 0;  biTri(2,3,2) = 1; biTri(2,4,2) = 0;
biTri(3,1,2) = 0; biTri(3,2,2) = 1;  biTri(3,3,2) = 0; biTri(3,4,2) = 0.108;
biTri(4,1,2) = 0; biTri(4,2,2) = 0;  biTri(4,3,2) = 0; biTri(4,4,2) = 1;

biTri(1,1,3) = 0; biTri(1,2,3) = 0; biTri(1,3,3) = 1;  biTri(1,4,3) = 0.105;
biTri(2,1,3) = -1; biTri(2,2,3) = 0; biTri(2,3,3) = 0;  biTri(2,4,3) = 0;
biTri(3,1,3) = 0; biTri(3,2,3) = -1; biTri(3,3,3) = 0;  biTri(3,4,3) = 0;
biTri(4,1,3) = 0; biTri(4,2,3) = 0; biTri(4,3,3) = 0;  biTri(4,4,3) = 1;

biTri(1,1,4) = 0; biTri(1,2,4) = 1;  biTri(1,3,4) = 0; biTri(1,4,4) = -0.1455;
biTri(2,1,4) = 0; biTri(2,2,4) = 0;  biTri(2,3,4) = -1; biTri(2,4,4) = 0;
biTri(3,1,4) = -1; biTri(3,2,4) = 0;  biTri(3,3,4) = 0; biTri(3,4,4) = 0.3265;
biTri(4,1,4) = 0; biTri(4,2,4) = 0;  biTri(4,3,4) = 0; biTri(4,4,4) = 1;

biTri(1,1,5) = 0; biTri(1,2,5) = 0; biTri(1,3,5) = 1;  biTri(1,4,5) = 0.095;
biTri(2,1,5) = 0; biTri(2,2,5) = -1; biTri(2,3,5) = 0; biTri(2,4,5) = 0;
biTri(3,1,5) = 1; biTri(3,2,5) = 0; biTri(3,3,5) = 0;  biTri(3,4,5) = 0;
biTri(4,1,5) = 0; biTri(4,2,5) = 0; biTri(4,3,5) = 0;  biTri(4,4,5) = 1;

biTri(1,1,6) = 0; biTri(1,2,6) = 0;  biTri(1,3,6) = 1; biTri(1,4,6) = 0;
biTri(2,1,6) = 0; biTri(2,2,6) = -1;  biTri(2,3,6) = 0; biTri(2,4,6) = 0;
biTri(3,1,6) = 1; biTri(3,2,6) = 0;  biTri(3,3,6) = 0; biTri(3,4,6) = 0.325;
biTri(4,1,6) = 0; biTri(4,2,6) = 0;  biTri(4,3,6) = 0; biTri(4,4,6) = 1;

numberOfJoints = size (biTri, 3);

%Initial configuration of the manipulator
qi = [0; 0; 0; 0; 0; 0]; 
% Vector containing the type for every link: 0-revolute, 1-prismatic
% for this manipulator all links are revolute
linkType = [0, 0, 0, 0, 0, 0];

%Computing initial transformation matrix
current_biTri = GetDirectGeometry(qi, biTri, linkType);

% Transformation matrix of the goal frame with respect to the base 
% (was chosen by us)
Tgoal = [1, 0, 0, 0.4;
         0, 1, 0, -0.3;
         0, 0, 1, 0.3;
         0, 0, 0,  1]; 
% Angular gain (was chosen by us)
gamma_a = 0.6; 
% Linear gain (was chosen by us)
gamma_l = 0.7; 

%Variables initialization
basicVectors = zeros(3, numberOfJoints);
distA = zeros(3, 1);
J = zeros(6, numberOfJoints);
% setting the state q at the initial state qi
q = qi;
qmin = zeros(1, numberOfJoints);
qmax = zeros(1, numberOfJoints);
% angular velocity
XA = zeros(3, 1);
% linear velocity
XL = zeros(3, 1);
% sample time
ts = 0.1;
% end time
t_end = 8.0;

for i = 1 : numberOfJoints
   % Revolute joint 
   if(linkType(i) == 0) 
       qmin(i) = -pi;
       qmax(i) = pi;
   % Prismatic joint    
   else 
       qmin(i) = 0;
       qmax(i) = 0.1;
   end
end

%% Kinematic Simulation

for i = 0.0:ts:t_end 
    
    % Here we compute the distance between the goal and the end effector
    distL = Tgoal(1:3, 4) - GetBasicVectorWrtBase(current_biTri, numberOfJoints);
    % Here we compute linear velocity of the end effector
    XL = gamma_l * distL;
    
    % Transformation of the end effector with respect to the base
    bTe = GetTransformationWrtBase(current_biTri, numberOfJoints);
    % Here we compute the angular distance between the goal and the end effector
    distA = VersorLemma(bTe(1:3, 1:3), Tgoal(1:3, 1:3));
    % Here we compute the angular velocity of the end effector
    XA = gamma_a * distA; 
    
    % vector for angular and linear velocity
    x = [XA; XL];
    % Here we compute the jacobian matrix thanks to the function
    % GetJacobian
    J = GetJacobian(current_biTri, bTe, linkType);
    % Here we compute joints's velocities corresponding to the desired velocities
    % of the end-effector
    q_vel = pinv(J)*x;
    
    % finally thanks to the KinematicSimulation function we can simulate
    % the robot behavior
    q = KinematicSimulation(q, q_vel,ts, qmin, qmax);
    
    current_biTri = GetDirectGeometry(q, biTri, linkType);
    
    for linkNumber = 1 : numberOfJoints
        basicVectors(:, linkNumber) = GetBasicVectorWrtBase(current_biTri, linkNumber);
    end

    hold on;

    % specifying view(3) we set the default 3D view in the plot
    view(3);
    
    %Plot basic vectors
    plot3(Tgoal(1,4), Tgoal(2,4),Tgoal(3,4),'greeno', 'LineWidth', 3);
    xPlot = [0, basicVectors(1,:)];
    yPlot = [0, basicVectors(2,:)];
    zPlot = [0, basicVectors(3,:)];
    plot3(xPlot, yPlot, zPlot, 'bluex-');
    xlabel("X-axis");
    ylabel("Y-axis");
    zlabel("Z-axis");
    grid();
    axis([-1 1 -1 1 -1 1]);
    getframe;
end


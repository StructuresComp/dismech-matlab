clc
clear all

fprintf('Discrete elastic rods (fully working!!)\n');


%%
global g dt tol

% Time step
dt = 1e-2;

% Density
rho = 1200;

% Cross-sectional radius of rod
r0 = 1e-3;  

% Thickness of shell
thickness = 1e-3;

% Young's modulus of rod
Y_Rod = 2e8;

% Young's modulus of shell
Y_Shell = 2e8;

% Poisson ratio
nu_Rod = 0.5;
nu_Shell = 0.5;

% gravity
gravity = 1; % or 0 for off

if (gravity==1)
    g = [0, 0, -9.81]';
else
    g = [0, 0, 0]';
end

% Maximum number of iterations in Newton Solver
maximum_iter = 100;

% Total simulation time (it exits after t=totalTime)
totalTime = 1; % sec

% Indicate whether images should be saved
saveImage = 0;

% How often the plot should be saved? (Set plotStep to 1 to show each plot)
plotStep = 1;

%% Inputs
% inputFileName = 'input_rod.txt';
% inputFileName = 'input_rod_straight.txt';
% inputFileName = 'input_-ve_cantilever_rod.txt';
inputFileName = 'input_rod_shell.txt';
% inputFileName = 'input_rod_shell - simpler.txt';
% inputFileName = 'input_shell_cantilever.txt';
% inputFileName = 'input_2rods_joint.txt';                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
% inputFileName = 'input_4node_Tjoint.txt';
% inputFileName = 'inp_4node_T_2b.txt';
% inputFileName = 'inp_1.txt';
% inputFileName = 'input_symmetric3rodJoint.txt';
% inputFileName = 'input_symmetric3rodJoint3BendSp.txt';
% inputFileName = 'inp_simplest_symmetric3.txt';
% inputFileName = 'input_symmetric3rodJoint6BendSpTooMany.txt';
% inputFileName = 'input_symmetric5rodJoint.txt';
% inputFileName = 'input_closed_square_loop.txt';

[nodes, edges, face_nodes, elStretchRod, elBendRod, elBendSign, elStretchShell, elBendShell] = inputProcessor(inputFileName);
n_nodes = size(nodes,1);
n_edges = size(edges,1);
n_stretch_rod = size(elStretchRod,1);
n_stretch_shell = size(elStretchShell,1);
assert(n_edges == n_stretch_rod + n_stretch_shell);
n_hinge = size(elBendShell,1);

twist_angles=zeros(n_stretch_rod,1);

face_nodes_shell = face_nodes;

%% for debugging
global bug
bug=0;

%% Create Discrete Rod struct (multiple rods)

global MultiRod
MultiRod = makeMultiRod(MultiRod, nodes, face_nodes_shell, twist_angles, edges, r0, thickness, rho, Y_Rod, nu_Rod, Y_Shell, nu_Shell);

%% Tolerance on force function. 

tol = 1e-3;
% tol = MultiRod.EI * 1e-6/ (MultiRod.refLen(1) ^ 2); % range 1e-8!

%% Creating stretching, bending, twisting and hinge springs

n_stretch = n_stretch_rod + n_stretch_shell;
n_bend_twist = size(elBendRod,1);

%% stretching spring

global stretch_spring_sample % stretch_springs

stretch_spring_sample = CreateStretchSpring(stretch_spring_sample, 0, [1 2]);

stretch_springs = repmat(stretch_spring_sample, n_stretch, 1); % array of stretch_spring

for s=1:n_stretch
    if (s > MultiRod.n_edges_rod) % shell stretching
        stretch_springs (s) = CreateStretchSpring (stretch_springs(s), ...
        MultiRod.refLen(s), elStretchShell(s-MultiRod.n_edges_rod,:), MultiRod.ks(s-MultiRod.n_edges_rod));
    else % rod stretching
    stretch_springs (s) = CreateStretchSpring (stretch_springs(s), ...
        MultiRod.refLen(s), elStretchRod(s,:));
    end
end


%% bending and twisting spring

if isempty(elBendSign)
    bend_twist_spring_sign = ones(n_bend_twist,2);
else
    bend_twist_spring_sign = elBendSign;
end

global bend_twist_spring_sample

bend_twist_spring_sample = CreateBendTwistSpring(bend_twist_spring_sample,...
    [1 2 3 1 2], [1 -1], zeros(n_nodes,1), [0,0], 0);

bend_twist_springs = repmat(bend_twist_spring_sample, n_bend_twist, 1); % array of bend_spring

for b=1:n_bend_twist
    bend_twist_springs(b) = CreateBendTwistSpring (bend_twist_springs(b), ...
        elBendRod(b,:), bend_twist_spring_sign(b,:), MultiRod.voronoiRefLen, [0 0], 0, [MultiRod.EI, MultiRod.GJ]);
end

%% hinge bending spring

global hinge_spring_sample % hinge_springs

hinge_spring_sample = CreateHingeSpring(hinge_spring_sample, 0, [1 2 3 4]);

hinge_springs = repmat(hinge_spring_sample, n_hinge, 1); % array of stretch_spring

for h=1:n_hinge
    hinge_springs(h) = CreateHingeSpring (hinge_springs(h), ...
    0, elBendShell(h,:), MultiRod.kb);
end

%% Rod related computations

% Reference frame (Space parallel transport at t=0)
MultiRod = ComputeSpaceParallel(MultiRod);

% Material frame
MultiRod.theta = MultiRod.q0(3*MultiRod.n_nodes+1:3*MultiRod.n_nodes+MultiRod.n_edges_rod); % twist angle
[MultiRod.m1, MultiRod.m2] = computeMaterialDirectors(MultiRod.a1,MultiRod.a2,MultiRod.theta); 
% this function will give the material directors from reference frame and twist angle 

% Reference twist
% refTwist_init = zeros(n_bend_twist, 1); % zero because initialized a1, a2 using space parallel transport
refTwist_init = computeRefTwist_bend_twist_spring ...
    (bend_twist_springs, MultiRod.a1, MultiRod.tangent, ...
     zeros(n_bend_twist,1));
MultiRod.refTwist = refTwist_init;

% Natural curvature
bend_twist_springs = setkappa(MultiRod, bend_twist_springs);

%% Fixed and Free DOFs

% Boundary Conditions
% MultiRod.fixed_nodes=[1, 2]; % required input
% MultiRod.fixed_nodes=[1, 4, 5, 6, 7, 8, 40, 73, 76, 78, 80]; % shell only
% MultiRod.fixed_nodes=[1, 2, 53, 54, 55, 56, 57, 89, 122, 125, 127, 129]; % rod + shell

MultiRod.fixed_nodes=[51, 52, 71, 72, 73, 74, 75, 94, 96, 98, 100]; % rod + shell another fixed at farther end of shell
MultiRod.fixed_edges=[]; % required input

[MultiRod.fixedDOF, MultiRod.freeDOF]=FindFixedFreeDOF(MultiRod);

% Visualize initial configuration and the fixed and free nodes: free nodes - blue, fixed - red
figure(1)
for i=1:n_edges
    n1 = MultiRod.Edges(i,1);
    n2 = MultiRod.Edges(i,2);
    n1pos = MultiRod.q(mapNodetoDOF(n1));
    n2pos = MultiRod.q(mapNodetoDOF(n2));
    plot3([n1pos(1);n2pos(1)], [n1pos(2);n2pos(2)], [n1pos(3);n2pos(3)],'bo-');
    hold on
end
plot3(MultiRod.Nodes(MultiRod.fixed_nodes,1), ...
    MultiRod.Nodes(MultiRod.fixed_nodes,2), MultiRod.Nodes(MultiRod.fixed_nodes,3),'ro')
hold off


%% Time stepping scheme

Nsteps = round(totalTime/dt);
ctime = 0; % current time (utility variable)

for timeStep = 1:Nsteps
    
    [MultiRod, stretch_springs, bend_twist_springs, hinge_springs] = DERfun_struct_new(MultiRod, stretch_springs, bend_twist_springs, hinge_springs);

    ctime = ctime + dt
    
    % Update q
    MultiRod.q0 = MultiRod.q;
    
     if mod(timeStep, plotStep) == 0
        MultiRod.theta = MultiRod.q((3*MultiRod.n_nodes+1):end);

        % visualisation
        plot_MultiRod(MultiRod, ctime);
%         axis equal
     end
end


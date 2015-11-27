function project

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN CHANGE
%

% - This line says your name.
params.teamname = 'My Team (My Name and My Other Name)';
% - This line says the file to record your actions.
params.action_filename = 'action.mat';
% - This line says the file to record your movie.
params.movie_filename = 'movie.avi';
% - This line says whether or not you want to record a movie --- change it
%   from "false" to "true" and you will record a movie. Note that you must
%   have already recorded actions, before making a movie!
params.makemovie = false;
% - This line says whether or not you want to use the keypad to turn on and
%   off the reaction wheels.
%
%   If you do, then:
%4
%       7,4,1 apply positive, zero, and negative torque to RW4
%       8,5,2 apply positive, zero, and negative torque to RW5
%
%   If you don't, then:
%
%       4,r,f apply positive, zero, and negative torque to RW4
%       5,t,g apply positive, zero, and negative torque to RW5
%
%   In either case, 'q' causes the simulation to quit (and to do it
%   gracefully, saving either the recorded actions or the movie).
params.usekeypad = false;

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN'T CHANGE
%

% Define the geometry of the spacecraft and both reaction wheels.
robot = GetGeometryOfRobot;

% Run the simulation.
RunSimulation(robot,params);
end





function robot = GetGeometryOfRobot
% - Vertices of spacecraft
robot.sc.dx=6;
robot.sc.dy=4;
robot.sc.dz=1;
robot.sc.p_in3 = [0.5*robot.sc.dx*[-1 1 1 -1 -1 1 1 -1];
                  0.5*robot.sc.dy*[-1 -1 -1 -1 1 1 1 1];
                  0.5*robot.sc.dz*[-1 -1 1 1 -1 -1 1 1]];

% - Vertices of reaction wheels
robot.rw4.dx=2;
robot.rw4.dy=1.5;
robot.rw4.dz=1;
robot.rw4.p_in4 = [0.5*robot.rw4.dx*[-1 1 1 -1 -1 1 1 -1];
                   0.5*robot.rw4.dy*[-1 -1 -1 -1 1 1 1 1];
                   0.5*robot.rw4.dz*[-1 -1 1 1 -1 -1 1 1]];
robot.rw5.dx=1;
robot.rw5.dy=2;
robot.rw5.dz=1.5;
robot.rw5.p_in5 = [0.5*robot.rw5.dx*[-1 1 1 -1 -1 1 1 -1];
                   0.5*robot.rw5.dy*[-1 -1 -1 -1 1 1 1 1];
                   0.5*robot.rw5.dz*[-1 -1 1 1 -1 -1 1 1]];

% - Faces for SC and RWs (all are boxes with vertices in the same order)
robot.faces =  [1 2 3;
                3 4 1;
                2 6 7;
                7 3 2;
                6 5 8;
                8 7 6;
                5 1 4;
                4 8 5;
                4 3 7;
                7 8 4;
                5 6 2;
                2 1 5];

% - The density (assumed uniform) of the SC and each RW
robot.sc.rho = 1;
robot.rw4.rho = 1;
robot.rw5.rho = 1;

% - The point at which each RW is attached to the SC
robot.rw4.ppin_in3 = [0;0.5*robot.sc.dy;0];
robot.rw5.ppin_in3 = [0;0;0.5*robot.sc.dz];
robot.rw4.ppin_in4 = [0;-0.5*robot.rw4.dy;0];
robot.rw5.ppin_in5 = [0;0;-0.5*robot.rw5.dz];

% - The CM of each RW
robot.o_4in3 = [0; 0.5*robot.sc.dy+0.5*robot.rw4.dy; 0];
robot.o_5in3 = [0; 0; 0.5*robot.sc.dz+0.5*robot.rw5.dz];

% - Magnitude of torque applied by SC on each RW
robot.rw4.tau = 10;
robot.rw5.tau = 10;

% - Create these for later use.
robot.sc.p_in0 = nan(size(robot.sc.p_in3));
robot.rw4.p_in0 = nan(size(robot.rw4.p_in4));
robot.rw5.p_in0 = nan(size(robot.rw5.p_in5));

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%%%
%
% MUST CHANGE
%

% - Mass and moment of inertia of SC
[robot.sc.m,robot.sc.J_in3] = MassAndMomentOfInertiaOfBox(robot.sc.dx,robot.sc.dy,robot.sc.dz);

% - Mass and moment of inertia of RW#4
[robot.rw4.m,robot.rw4.J_in4] = MassAndMomentOfInertiaOfBox(robot.rw4.dx,robot.rw4.dy,robot.rw4.dz);

% - Mass and moment of inertia of RW#5
[robot.rw5.m,robot.rw5.J_in5] = MassAndMomentOfInertiaOfBox(robot.rw5.dx,robot.rw5.dy,robot.rw5.dz);




% ROBOT ARM ROBOT ARM ROBOT ARM ROBOT ARM ROBOT ARM
% - Vertices for base
robot.base.dx=sqrt(2);
robot.base.dy=sqrt(2);
robot.base.dz=0.5;
robot.base.p_in0 = [0.5*robot.base.dx*[-1 1 1 -1 -1 1 1 -1];
                    0.5*robot.base.dy*[-1 -1 -1 -1 1 1 1 1];
                    0.5*robot.base.dz*[-1 -1 1 1 -1 -1 1 1]];
% - Vertices of link #1
robot.link1.dx=2;
robot.link1.dy=2;
robot.link1.dz=1.5;
robot.link1.p_in1 = [0.5*robot.link1.dx*[-1 1 1 -1 -1 1 1 -1];
                     0.5*robot.link1.dy*[-1 -1 -1 -1 1 1 1 1];
                     0.5*robot.link1.dz*[-1 -1 1 1 -1 -1 1 1]];
% - Vertices of link #2
robot.link2.dx=1;
robot.link2.dy=3;
robot.link2.dz=1;
robot.link2.p_in2 = [0.5*robot.link2.dx*[-1 1 1 -1 -1 1 1 -1];
                     0.5*robot.link2.dy*[-1 -1 -1 -1 1 1 1 1];
                     0.5*robot.link2.dz*[-1 -1 1 1 -1 -1 1 1]];
% - Vertices of link #3
robot.link3.dx=1;
robot.link3.dy=4;
robot.link3.dz=1;
robot.link3.p_in3 = [0.5*robot.link3.dx*[-1 1 1 -1 -1 1 1 -1];
                     0.5*robot.link3.dy*[-1 -1 -1 -1 1 1 1 1];
                     0.5*robot.link3.dz*[-1 -1 1 1 -1 -1 1 1]];
% - Create for later use
robot.link1.p_in0 = nan(size(robot.link1.p_in1));
robot.link2.p_in0 = nan(size(robot.link2.p_in2));
robot.link3.p_in0 = nan(size(robot.link3.p_in3));
% - Faces for base and for each link (all are boxes with vertices in the
%   same order)
robot.faces =  [1 2 3;
                3 4 1;
                2 6 7;
                7 3 2;
                6 5 8;
                8 7 6;
                5 1 4;
                4 8 5;
                4 3 7;
                7 8 4;
                5 6 2;
                2 1 5];

% - The density (assumed uniform) of each link
robot.link1.rho = 1;
robot.link2.rho = 1;
robot.link3.rho = 1;

% - Parameters that you may ignore
alpha = 0.5;
beta = 0.35;
delta = 0.05;

% - Parameters that govern the relative position of each frame
%
%       o_10 = robot.a1 + R_1in0*robot.b1
%
%           ... (same for the others)
%
robot.a1 = [0; 0; (alpha+delta)*(robot.link1.dz+robot.base.dz)];
robot.b1 = [0; 0; 0];
robot.a2 = [(alpha+delta)*(robot.link2.dx+robot.link1.dx); 0; 0];
robot.b2 = [0; beta*robot.link2.dy; 0];
robot.a3 = [(alpha+delta)*(robot.link3.dx+robot.link2.dx); beta*robot.link2.dy; 0];
robot.b3 = [0; beta*robot.link3.dy; 0];

% - The points of attachment between links
%
%       p_ABinC means the point of attachment between link A and link B,
%               written in the coordinates of frame C (where "link 0" and
%               "base" are the same thing)
%
robot.p_01in0 = [0;0;(alpha+delta)*robot.base.dz];
robot.p_12in1 = [(alpha+delta)*robot.link1.dx;0;0];
robot.p_23in2 = [(alpha+delta)*robot.link2.dx;beta*robot.link2.dy;0];

% - The coefficient of friction at each joint
robot.kfriction = 1;

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MUST CHANGE
%

% - Mass and moment of inertia of link #1
[robot.link1.m,robot.link1.J_in1] = MassAndMomentOfInertiaOfBox(robot.link1.dx,robot.link1.dy,robot.link1.dz);

% - Mass and moment of inertia of link #2
[robot.link2.m,robot.link2.J_in2] = MassAndMomentOfInertiaOfBox(robot.link2.dx,robot.link2.dy,robot.link2.dz);

% - Mass and moment of inertia of link #3
[robot.link3.m,robot.link3.J_in3] = MassAndMomentOfInertiaOfBox(robot.link3.dx,robot.link3.dy,robot.link3.dz);
end





function RunSimulation(robot,params)

% Create figure.
world = CreateFigure(robot,params);
% Define time at which to start the simulation.
t = 0;
% Define time step.
dt = 2e-2;


% Define time limit.
tmax = 100;
% Define initial conditions
% - configuration
o_3in0 = [0;0;0];
theta = [0;0;0];
phi4 = 0;
phi5 = 0;
% - velocity
v_03in0 = [0;0;0];
w_03in3 = [0;0;0];
phi4dot = 0;
phi5dot = 0;
% Define initial conditions
% - joint angles (configuration)
theta = [0;0;0];
% - joint velocities
thetadot = [0;0;0];
% Define intial motor torques
u1 = 0;                                     % CHANGED FOR NEW SYSTEM 1,2,3 robot 4,5 sc
u2 = 0;
u3 = 0;
u4 = 0;
u5 = 0;

% Either start making a movie or start storing actions.
global action done
action = [u1;u2;u3;u4;u5];                 % CHANGED FOR NEW SYSTEM 1,2,3 robot 4,5 sc
if (params.makemovie)
    load(params.action_filename);
    myV = VideoWriter(params.movie_filename);
    myV.Quality = 100;
    open(myV);
else
    actionRecord = [];
end

% Loop until 'done' is true.
done = false;
while (~done)
    
    % Either store or retrieve the action.
    if (params.makemovie)
        [actionRecord,curaction,done] = RetrieveAction(actionRecord);
        action = curaction;
    else
        curaction = action;
        actionRecord = StoreAction(actionRecord,curaction);
    end
    
    % Compute input torques applied by SC to each RW.
    u1 = curaction(1);
    u2 = curaction(2);
    u3 = curaction(3);
    u4 = curaction(4)*robot.rw4.tau;
    u5 = curaction(5)*robot.rw5.tau;
	
    % Solve ODEs      |ROBOT ARM
    [t,theta,thetadot,o_3in0,theta,phi4,phi5,v_03in0,w_03in3,phi4dot,phi5dot] = ...
        Simulate(t,dt,...           % <- time and time step
                 theta,thetadot,... % <- state
                 u1,u2,u3,...       % <- applied torques
                 robot,...            % <- parameters that describe the robot
                 o_3in0,theta,phi4,phi5,...             % <- configuration
                 v_03in0,w_03in3,phi4dot,phi5dot,...    % <- velocity
                 u4,u5);                              % <- applied torques

       % ------------------------------------------------------------------
    % ROBOT
    %
    
    % Compute: orientation and position of link #1
    %   (R_1in0 and o_1in0)
    R_1in0 = RZ(theta(1));
    o_1in0 = robot.a1+R_1in0*robot.b1;
    
    % Compute: orientation and position of link #2
    %   (R_2in0 and o_2in0 ... hint: first compute R_2in1 and o_2in1)
    R_2in1 = RX(theta(2));
    o_2in1 = robot.a2+R_2in1*robot.b2;
    R_2in0 = R_1in0*R_2in1;
    o_2in0 = o_1in0+R_1in0*o_2in1;
    
    % Compute: orientation and position of link #3
    %   (R_3in0 and o_3in0 ... hint: first compute R_3in2 and o_3in2)
    R_3in2 = RX(theta(3));
    o_3in2 = robot.a3+R_3in2*robot.b3;
    R_3in0 = R_2in0*R_3in2;
    o_3in0 = o_2in0+R_2in0*o_3in2;
    
    % Compute: robot.link1.p_in0, robot.link2.p_in0, robot.link3.p_in0
    for i=1:size(robot.link1.p_in1,2)
        robot.link1.p_in0(:,i) = o_1in0+R_1in0*robot.link1.p_in1(:,i);
    end
    for i=1:size(robot.link2.p_in2,2)
        robot.link2.p_in0(:,i) = o_2in0+R_2in0*robot.link2.p_in2(:,i);
    end
    for i=1:size(robot.link3.p_in3,2)
        robot.link3.p_in0(:,i) = o_3in0+R_3in0*robot.link3.p_in3(:,i);
    end
    
    % SC
    %
    
    % Compute orientation of 3 in 0
    %  (hint: you have theta - assume XYZ Euler Angle sequence, as usual)
    R_3in0 = GetR_3in0(theta);
    
    % Compute position and orientation of 4 and 5 in 0
    %  (hint: you have o_3in0, R_3in0, phi4, phi5,
    %                  robot.o_4in3, robot.o_5in3)
    o_4in0 = o_3in0 + R_3in0*robot.o_4in3;
    o_5in0 = o_3in0 + R_3in0*robot.o_5in3;
    R_4in0 = R_3in0*RY(phi4);
    R_5in0 = R_3in0*RZ(phi5);
    
    % Compute robot.sc.p_in0, robot.rw4.p_in0, robot.rw5.p_in0
    %  (hint: you have o_3in0, R_3in0, o_4in0, R_4in0, o_5in0, R_5in0,
    %                  robot.sc.p_in3, robot.rw4.p_in4, robot.rw5.p_in5)
    for i=1:size(robot.sc.p_in3,2)
        robot.sc.p_in0(:,i) = o_3in0 + R_3in0*robot.sc.p_in3(:,i);
    end
    for i=1:size(robot.rw4.p_in4,2)
        robot.rw4.p_in0(:,i) = o_4in0 + R_4in0*robot.rw4.p_in4(:,i);
    end
    for i=1:size(robot.rw5.p_in5,2)
        robot.rw5.p_in0(:,i) = o_5in0 + R_5in0*robot.rw5.p_in5(:,i);
    end
    
    
    v_04in0 = v_03in0+R_3in0*wedge(w_03in3)*robot.o_4in3;
    v_05in0 = v_03in0+R_3in0*wedge(w_03in3)*robot.o_5in3;
    
    
    %--------------------------------------------------------------------
    
    
    
        % Update the figure.
    world = UpdateFigure(world,robot,o_1in0,R_1in0,o_2in0,R_2in0,o_3in0,R_3in0,u1,u2,u3,t,tmax);
    
    % If making a movie, store the current figure as a frame.
    if (params.makemovie)
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    
    % Stop if time has reached its maximum.
    if (t>tmax)
        done = true;
    end
    
end
% Either close the movie or save the record of actions.
if (params.makemovie)
    for i=1:30
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
else
    save(params.action_filename,'actionRecord');
end

if (params.makesnapshot)
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',params.snapshot_filename);
end
    
end




function world = CreateFigure(robot,params)
% - clear the current figure
clf;
% - these things have the correct value
o_0in0 = zeros(3,1);
R_0in0 = eye(3);
% - these things have the incorrect value, but we don't care
o_3in0 = zeros(3,1);
R_3in0 = eye(3);
% - text (it's important this is in the back, so you can rotate the view
%         and other stuff!)
axes('position',[0 0 1 1]);
axis([0 1 0 1]);
hold on;
axis off;
fs = 10;
world.text.time=text(0.05,0.1,sprintf('t = %6.2f / %6.2f\n',0,0),'fontsize',fs,'verticalalignment','top','fontname','monaco');
world.text.teamname=text(0.05,0.04,params.teamname,'fontsize',fs,'verticalalignment','top','fontweight','bold');
% - view from frame 0
axes('position',[0.05 -0.05 .9 1.15]);
title('view: frame 0                                                 ');
set(gcf,'renderer','opengl');
axis equal;
axis(3*[-2 2 -2 2 -1 1]);
axis manual;
hold on;
view([90-37.5,20]);
box on;
set(gca,'projection','perspective');
set(gca,'clipping','on','clippingstyle','3dbox');
world.view0.robot = DrawRobot([],robot,o_3in0,R_3in0,0.6);
world.view0.frame0 = DrawFrame([],o_0in0,R_0in0);
world.view0.frame1 = DrawFrame([],zeros(3,1),eye(3));
world.view0.frame2 = DrawFrame([],zeros(3,1),eye(3));
world.view0.frame6 = DrawFrame([],zeros(3,1),eye(3));
world.view0.frame3 = DrawFrame([],o_3in0,R_3in0);
lighting gouraud
world.view0.light = light('position',o_3in0','style','local');
% - make the figure respond to key commands
if (params.usekeypad)
    set(gcf,'KeyPressFcn',@onkeypress_keypad);
else
    set(gcf,'KeyPressFcn',@onkeypress_nokeypad);
end
end






function actionRecord = StoreAction(actionRecord,action)
actionRecord(:,end+1) = action;
end

function [actionRecord,action,done] = RetrieveAction(actionRecord)
action = actionRecord(:,1);
actionRecord = actionRecord(:,2:end);
done = isempty(actionRecord);
end

function [m,J] = MassAndMomentOfInertiaOfBox(x,y,z)
% - assumes unit, uniform density
m = x*y*z;
J = (m/12)*diag([y^2+z^2,z^2+x^2,x^2+y^2]);
end



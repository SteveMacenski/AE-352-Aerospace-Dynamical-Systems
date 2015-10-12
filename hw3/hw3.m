function hw3

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% INITIALIZATION
%

%%%%
% STUFF YOU SHOULD NOT CHANGE
%
% Define a global variable.
global thruster
% Define the geometry of everything.
[robot,thruster] = GetGeometry;
% Create figure.
world = CreateFigure(robot,thruster);
% Define time at which to start the simulation.
t = 0;
% Define time step.
dt = 1e-2;
%
%%%%

%%%%
% STUFF YOU CAN CHANGE IF YOU WANT TO
%
% Define initial conditions.
%
%   o_3in0  the position of frame 3 in the coordinates of frame 0 at the
%           start of the simulation
%
%   theta   3x1 matrix with the XYZ Euler Angles that describe the
%           orientation of frame 3 with respect to frame 0 at the start of
%           the simulation
%
%   v_03in0 the linear velocity of frame 3 with respect to frame 0 in the
%           coordinates of frame 0 at the start of the simulation
%
%   w_03in3 the angular velocity of frame 3 with respect to frame 0 in the
%           coordinates of frame 3 at the start of the simulation
%
o_3in0 = [0;-4;1];
theta = [0;0;pi/2];
v_03in0 = zeros(3,1);
w_03in3 = zeros(3,1);
%
% Define time at which to stop the simulation.
%
%   tmax    a number, the time at which the simulation will end
%
tmax = 10;
%
%%%%

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% SIMULATION LOOP
%

while (t<=tmax)
    
    %%%%
    % STUFF YOU SHOULD NOT CHANGE
    %
    % Solve ODEs to find position and orientation after time step.
    [t,o_3in0,theta,v_03in0,w_03in3] = Simulate(t,o_3in0,theta,v_03in0,w_03in3,robot,thruster,dt);
    %
    %%%%
    
    %%%%
    % STUFF YOU MUST CHANGE
    %
    % (1) Compute position and orientation of 3 in 0 and of 0 in 3
    %
    % Given:
    %
    %   o_3in0  the position of frame 3 in the coordinates of frame 0
    %
    %   theta   a 3x1 matrix of XYZ Euler Angles in radians
    %
    % Compute:
    %
    %   R_3in0  the orientation of frame 3 in the coordinates of frame 0
    %           (this comes from XYZ Euler Angle sequence)
    %
theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);
R_3in0 = [cos(theta2)*cos(theta3), -cos(theta2)*sin(theta3), sin(theta2);... 
          cos(theta1)*sin(theta3)+cos(theta3)*sin(theta1)*sin(theta2), cos(theta1)*cos(theta3)-sin(theta1)*sin(theta2)*sin(theta3), -cos(theta2)*sin(theta1);...
          sin(theta1)*sin(theta3)-cos(theta1)*cos(theta3)*sin(theta2), cos(theta3)*sin(theta1)+cos(theta1)*cos(theta1)*sin(theta2)*sin(theta3), cos(theta1)*cos(theta2)];    % <-- CHANGE THIS LINE (PROBLEM 3(a))
    %   
    %;    % <-- CHANGE THIS LINE (PROBLEM 7(a))
    %
    %   R_0in3  the orientation of frame 0 in the coordinates of frame 3
    %           (this comes from an inverse transformation)
    %
    R_0in3 = transpose(R_3in0);    % <-- CHANGE THIS LINE (PROBLEM 7(b))
    %
    %   o_0in3  the position of frame 0 in the coordinates of frame 3
    %           (this comes from an inverse transformation)
    %
    o_0in3 = -R_0in3*o_3in0;   % <-- CHANGE THIS LINE (PROBLEM 7(b))
    %
    %
    %
    % (2) Transform stuff from frame 3 to frame 0
    %
    % Given:
    %
    %   o_3in0  the position of frame 3 in the coordinates of frame 0
    %
    %   R_3in0  the orientation of frame 3 in the coordinates of frame 0
    %
    %   robot.p_in3, thruster(1).p_in3, thruster(2).p_in3, ...
    %       - all matrices of size 3 x <something>
    %       - each column has the coordinates of a point in frame 3
    %
    % Compute:
    %
    %   robot.p_in0, thruster(1).p_in0, thruster(2).p_in0, ...
    %       - all matrices of size 3 x <something>
    %       - each column has the coordinates of a point in frame 0
    %
    robot.p_in0 = R_3in0*robot.p_in3 + [o_3in0 o_3in0 o_3in0 o_3in0 o_3in0 o_3in0 o_3in0 o_3in0] ;                  % <-- CHANGE THIS LINE (PROBLEM 7(c))
    for i=1:length(thruster)
        thruster(i).p_in0 = R_3in0*thruster(i).p_in3 + o_3in0;   % <-- CHANGE THIS LINE (PROBLEM 7(c))
    end
    %
    %%%%
    
    %%%%
    % STUFF YOU SHOULD NOT CHANGE
    %
    % Update the figure.
    world = UpdateFigure(world,robot,thruster,o_3in0,R_3in0,o_0in3,R_0in3,t,tmax);
    %
    %%%%
    
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS YOU MUST CHANGE
%

function [o_3in0dot,thetadot,v_03in0dot,w_03in3dot] = GetRates(o_3in0,theta,v_03in0,w_03in3,m,J_in3,thruster)
%
% Given:
%
%   o_3in0      is the position of frame 3 in the coordinates of frame 0
%
%   theta       is a 3x1 matrix with the XYZ Euler Angles (in radians)
%
%   v_03in0     is the linear velocity of frame 3 with respect to frame 0,
%               written in the coordinates of frame 0
%
%   w_03in3     is the angular velocity of frame 3 with respect to frame 0,
%               written in the coordinates of frame 3
%
%   m           is the mass of the robot
%
%   J_in3          is the moment of inertia matrix of the robot in the
%               coordinates of frame 3
%
%   thruster    - is an array
%               - to find its length: length(thruster)
%               - to get i'th force, in the coordinates of frame 3:
%                   thruster(i).f_in3
%               - to get point of application of i'th force, in the
%                 coordinates of frame 3:
%                   thruster(i).p_in3
%               - to see if the i'th force is turned on:
%                   if (thruster(i).on)
%                       ... do something ...
%                   end
%
% Find:
%
%   o_3in0dot   is the time derivative of o_3in0
%
%   thetadot    is the time derivative of theta
%
%   v_03in0     is the time derivative of v_03in0
%
%   w_03in3     is the time derivative of w_03in3

theta1 = theta(1);
theta2 = theta(2);
theta3 = theta(3);
R_3in0 = [cos(theta2)*cos(theta3), -cos(theta2)*sin(theta3), sin(theta2);... 
          cos(theta1)*sin(theta3)+cos(theta3)*sin(theta1)*sin(theta2), cos(theta1)*cos(theta3)-sin(theta1)*sin(theta2)*sin(theta3), -cos(theta2)*sin(theta1);...
          sin(theta1)*sin(theta3)-cos(theta1)*cos(theta3)*sin(theta2), cos(theta3)*sin(theta1)+cos(theta1)*cos(theta1)*sin(theta2)*sin(theta3), cos(theta1)*cos(theta2)];    % <-- CHANGE THIS LINE (PROBLEM 3(a))
 
%
o_3in0dot = v_03in0;    
thetadot = [cos(theta3)/cos(theta2) -sin(theta3)/cos(theta2) 0;
            sin(theta3) cos(theta3) 0
            -sin(theta2)*cos(theta3)/cos(theta2) sin(theta2)*sin(theta3)/cos(theta2) 1]*w_03in3;     % <-- CHANGE THIS LINE (PROBLEM 7(d))

sumf = [0;0;0];
    for i= 1: length(thruster)
       if (thruster(i).on==true)
           sumf = sumf + thruster(i).f_in3;
       end
end
v_03in0dot = (R_3in0*sumf)/m ; % <-- CHANGE THIS LINE (PROBLEM 7(d))

sumt = [0;0;0];
for i = 1: length(thruster)
   if (thruster(i).on==true)
    p1 = thruster(i).p_in3(1);
    p2 = thruster(i).p_in3(2);
    p3 = thruster(i).p_in3(3);
    p3_wedge = [0 -p3 p2;
                p3 0 -p1;
                -p2 p1 0];
    sumt = sumt +  p3_wedge*thruster(i).f_in3;
   end
end

w1 = w_03in3(1);
    w2 = w_03in3(2);
    w3 = w_03in3(3);
    w3_wedged = [0 -w3 w2;
                w3 0 -w1;
                -w2 w1 0];
w_03in3dot = inv(J_in3)*(sumt - (w3_wedged*J_in3*w_03in3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS YOU SHOULD NOT CHANGE
%
function wedge = Wedge(vector)
wedge = [0 -vector(3) vector(2);
         vector(3) 0 -vector(1);
         -vector(2) vector(1) 0];
return

function [t,o_3in0,theta,v_03in0,w_03in3] = Simulate(t,o_3in0,theta,v_03in0,w_03in3,robot,thruster,dt)
[t,x] = ode45(@(t,x) GetXDot(t,x,robot,thruster),[t t+dt],[o_3in0;theta;v_03in0;w_03in3]);
x = x';
t = t(end);
o_3in0 = x(1:3,end);
theta = x(4:6,end);
v_03in0 = x(7:9,end);
w_03in3 = x(10:12,end);

function xdot = GetXDot(t,x,robot,thruster)
% Take apart the state.
o_3in0 = x(1:3,:);
theta = x(4:6,:);
v_03in0 = x(7:9,:);
w_03in3 = x(10:12,:);
% Get time derivatives of the state.
[o_3in0dot,thetadot,v_03in0dot,w_03in3dot] = GetRates(o_3in0,theta,v_03in0,w_03in3,robot.m,robot.J_in3,thruster);
% Put the state back together.
xdot = [o_3in0dot; thetadot; v_03in0dot; w_03in3dot];

function [robot,thruster] = GetGeometry
% - The spacecraft (a box).
robot.dx=1.25;
robot.dy=0.75;
robot.dz=0.25;
robot.p_in3 = [-robot.dx/2  robot.dx/2  robot.dx/2 -robot.dx/2 -robot.dx/2  robot.dx/2  robot.dx/2 -robot.dx/2;
               -robot.dy/2 -robot.dy/2 -robot.dy/2 -robot.dy/2  robot.dy/2  robot.dy/2  robot.dy/2  robot.dy/2;
               -robot.dz/2 -robot.dz/2  robot.dz/2  robot.dz/2 -robot.dz/2 -robot.dz/2  robot.dz/2  robot.dz/2];
robot.faces =  [1 2 3 4;
                2 6 7 3;
                6 5 8 7;
                5 1 4 8;
                4 3 7 8;
                5 6 2 1];
robot.pCM_in3 = zeros(3,1);
robot.p_in0 = nan(3,1);
robot.m = 1;
robot.J_in3 = (robot.m/12)*diag([(robot.dy^2+robot.dz^2),(robot.dz^2+robot.dx^2),(robot.dx^2+robot.dy^2)]);
% - The thrusters.
thruster = [];
thruster = AddThruster(thruster,[-robot.dx/2;0;0],[2;0;0]);
thruster = AddThruster(thruster,[-robot.dx/4;-robot.dy/2;0],[0;1;0]);
thruster = AddThruster(thruster,[-robot.dx/4;robot.dy/2;0],[0;-1;0]);
thruster = AddThruster(thruster,[-robot.dx/4;0;-robot.dz/2],[0;0;1]);
thruster = AddThruster(thruster,[-robot.dx/4;0;robot.dz/2],[0;0;-1]);

function thruster = AddThruster(thruster,p_in3,f_in3)
newthruster.p_in3 = p_in3;
newthruster.f_in3 = f_in3;
newthruster.on = false;
newthruster.p_in0 = nan(3,1);
newthruster.f_in0 = nan(3,1);
if (isempty(thruster))
    clear thruster;
    thruster(1) = newthruster;
else
    thruster(end+1) = newthruster;
end

function thruster = DrawThruster(thruster,p,color)
if (isempty(thruster))
    thruster = plot3(p(1),p(2),p(3),'marker','.','color',color,'markersize',16);
    else
    set(thruster,'xdata',p(1),'ydata',p(2),'zdata',p(3),'color',color);
end

function robot = DrawRobot(robot,p,f)
if isempty(robot)
    robot = patch('Vertices',p','Faces',f,...
                  'FaceColor','y','FaceAlpha',0.6,'EdgeAlpha',0.6);
else
    set(robot,'vertices',p');
end

function frame = DrawFrame(frame,o,R)
p = [o repmat(o,1,3)+R];
if isempty(frame)
    frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',2);
    frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',2);
    frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',2);
else
    set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
    set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
    set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));
end

function world = CreateFigure(robot,thruster)
% - clear the current figure
clf;
% - text
axes('position',[0 0 1 1]);
axis([0 1 0 1]);
hold on;
axis off;
fs = 10;
world.text.time=text(0.05,0.1,sprintf('t = %6.2f / %6.2f\n',0,0),'fontsize',fs,'verticalalignment','top','fontname','monaco');
% - placeholders (these things have the wrong value)
o_0in0 = zeros(3,1);
R_0in0 = eye(3);
o_3in0 = zeros(3,1);
R_3in0 = eye(3);
o_1in3 = zeros(3,1);
R_1in3 = eye(3);
o_0in3 = zeros(3,1);
R_0in3 = eye(3);
% - view from frame 0
axes('position',[0.05 -0.05 .9 1.15]);
title('view: frame 0                                                 ');
set(gcf,'renderer','opengl');
axis equal;
axis([-6 6 -6 6 -3 3]);
axis manual;
hold on;
view([90-37.5,20]);
box on;
set(gca,'projection','perspective');
lighting gouraud
for i=1:3
    world.view0.light(i) = light('position',(o_0in0+R_0in0(:,i))');
end
set(gca,'clipping','on','clippingstyle','rectangle');
world.view0.robot = DrawRobot([],robot.p_in0,robot.faces);
world.view0.frame0 = DrawFrame([],o_0in0,R_0in0);
world.view0.frame3 = DrawFrame([],o_3in0,R_3in0);
for i=1:length(thruster)
    world.view0.thruster(i) = DrawThruster([],thruster(i).p_in0,'k');
end
% - view from frame 3
axes('position',[0.53 0.73 0.445 0.23]);
title('view: frame 3');
axis off;
axis([0 1 0 1]);
axis manual;
hold on;
box on;
rectangle('position',[0 0 1 1],'facecolor','w','edgecolor','k');
set(gca,'clipping','off');
axes('position',[0.655 0.78 0.2 0.2]);
set(gcf,'renderer','opengl');
axis equal;
axis([-.7 1e3 -2.2 2.2 -0.2 1.5]);
axis manual;
hold on;
box on;
lighting gouraud
for i=1:3
    world.view1.light(i) = light('position',(o_3in0+R_3in0(:,i))');
end
set(gca,'projection','perspective');
campos(1.0*[-2;0;0.9]);
camtarget([1e3;0;0]);
camva(60);
axis off;
set(gca,'clipping','on','clippingstyle','rectangle');
world.view1.robot = DrawRobot([],robot.p_in3,robot.faces);
world.view1.frame0 = DrawFrame([],o_0in3,R_0in3);
world.view1.frame3 = DrawFrame([],o_1in3,R_1in3);
for i=1:length(thruster)
    world.view1.thruster(i) = DrawThruster([],thruster(i).p_in3,'k');
end
% - make the figure respond to key commands
set(gcf,'KeyPressFcn',@onkeypress);

function world = UpdateFigure(world,robot,thruster,o_3in0,R_3in0,o_0in3,R_0in3,t,tmax)
world.view0.robot = DrawRobot(world.view0.robot,robot.p_in0,robot.faces);
world.view0.frame3 = DrawFrame(world.view0.frame3,o_3in0,R_3in0);
world.view1.robot = DrawRobot(world.view1.robot,robot.p_in3,robot.faces);
world.view1.frame0 = DrawFrame(world.view1.frame0,o_0in3,R_0in3);
for i=1:3
    set(world.view1.light(i),'position',(o_3in0+R_3in0(:,i))');
end
for i=1:length(thruster)
    if (thruster(i).on)
        world.view0.thruster(i) = DrawThruster(world.view0.thruster(i),thruster(i).p_in0,[0.9 0.5 0]);
        world.view1.thruster(i) = DrawThruster(world.view1.thruster(i),thruster(i).p_in3,[0.9 0.5 0]);
    else
        world.view0.thruster(i) = DrawThruster(world.view0.thruster(i),thruster(i).p_in0,'k');
        world.view1.thruster(i) = DrawThruster(world.view1.thruster(i),thruster(i).p_in3,'k');
    end
end
set(world.text.time,'string',sprintf('t = %6.2f / %6.2f\n',t,tmax));
drawnow

function onkeypress(src,event)
global thruster
for i=1:length(thruster)
    if event.Character == char(48+i)
        thruster(i).on = ~thruster(i).on;
    end
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

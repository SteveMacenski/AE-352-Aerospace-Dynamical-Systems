function hw4

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CHANGE THESE FOUR LINES (IF YOU LIKE)
%
% - This line says your name.
params.teamname = 'Team Under 10 seconds (Steve Macenski & Chris Lorenz)';
% - This line says the file to record your thruster firings.
params.action_filename = 'action.mat';
% - This line says the file to record your movie.
params.movie_filename = 'Team Under 10 seconds movie_new.avi';
% - This line says whether or not you want to record a movie --- change it
%   from "false" to "true" and you will record a movie. Note that you must
%   have already recorded thruster firings, before making a movie!
params.makemovie = false;
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define the geometry of the spacecraft and its thrusters.
[robot,thruster] = GetGeometryOfRobot;

% Run the simulation.
RunSimulation(robot,thruster,params);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CHANGE THIS FUNCTION
%
function [robot,thruster] = GetGeometryOfRobot
% GEOMETRY OF THE SPACECRAFT
% - The following three lines define three parameters for my own use. If
%   you don't use them, you can delete them.
robot.dx=1;
robot.dy=1;
robot.dz=.5;
% - The following line defines the vertices of the spacecraft. The variable
%   "robot.p_in3" must be a matrix of size 3xn, where "n" is the number of
%   vertices. The i'th column of this matrix describes the i'th vertex in
%   the coordinates of frame 3. You must change this line to define the
%   vertices of your own spacecraft.
robot.p_in3 = [-robot.dx/2  robot.dx/2  robot.dx/2 -robot.dx/2 -robot.dx/2  robot.dx/2  robot.dx/2 -robot.dx/2;
               -robot.dy/2 -robot.dy/2 -robot.dy/2 -robot.dy/2  robot.dy/2  robot.dy/2  robot.dy/2  robot.dy/2;
               -robot.dz/2 -robot.dz/2  robot.dz/2  robot.dz/2 -robot.dz/2 -robot.dz/2  robot.dz/2  robot.dz/2];
% - The following line defines the "faces" of the spacecraft. The variable
%   "robot.faces" must be a matrix of size mx3, where "m" is the number of
%   faces. The j'th row of this matrix describes the j'th face. Since the
%   j'th face is a triangle, it can be described by listing its three
%   vertices. So, the j'th row of this matrix has three numbers -- each
%   number is the index of a vertex. For example, if the j'th row was
%   [2 5 3], then the vertices of the j'th face would be described by
%   columns 2, 5, and 3 of "robot.p_in3". Vertices must be listed in
%   counter-clockwise order, if you were looking at the triangle from the
%   outside. You must change this line to define the faces of your own
%   spacecraft.
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
% - The following line defines the mass of the spacecraft. You must change
%   this line to define the mass of your own spacecraft.
robot.m = robot.dx*robot.dy*robot.dz;
% - The following line defines the moment of inertia matrix of the
%   spacecraft. You must change this line to define the moment of inertia
%   matrix of your own spacecraft.
robot.J_in3 = (robot.m/12)*diag([(robot.dy^2+robot.dz^2),(robot.dz^2+robot.dx^2),(robot.dx^2+robot.dy^2)]);
% GEOMETRY OF THE THRUSTERS
% - The following line initializes the array of thrusters. Don't change it!
thruster = [];
% - Each of the following lines adds a thruster. To add your own thrusters,
%   use the same syntax:
%
%       thruster = AddThruster( thruster, <-- always the same
%                               3x1 matrix describing the point of
%                                   application in coordinates of frame 3,
%                               3x1 matrix describing force in coordinates
%                                   of frame 3 );
%
%   To remove a thruster, just delete one of these lines. You must add /
%   delete lines to define the thrusters for your own spacecraft.
%
thruster = AddThruster(thruster,[-robot.dx/2;0;0],[0.25;0;0]); %+z face
thruster = AddThruster(thruster,[robot.dx/2;0;0],[-0.25;0;0]); %-z face

thruster = AddThruster(thruster,[-robot.dx/4;-robot.dy/2;0],[0;0.25;0]);%-x face
thruster = AddThruster(thruster,[robot.dx/4;-robot.dy/2;0],[0;0.25;0]);

thruster = AddThruster(thruster,[robot.dx/4;robot.dy/2;0],[0;-0.25;0]);%+x face
thruster = AddThruster(thruster,[-robot.dx/4;robot.dy/2;0],[0;-0.25;0]);

thruster = AddThruster(thruster,[-robot.dx/4;0;-robot.dz/2],[0;0;0.25]); %-y face
thruster = AddThruster(thruster,[robot.dx/4;0;-robot.dz/2],[0;0;0.25]);

thruster = AddThruster(thruster,[-robot.dx/4;0;robot.dz/2],[0;0;-0.25]);%+y face
thruster = AddThruster(thruster,[robot.dx/4;0;robot.dz/2],[0;0;-0.25]);


% first move y (90 - 78) to the hole (at same time 1 - 2 for a little up bump)
%then (56-78) for +x motion through hole
% THEN (07- 89) to align y rotation #1
%make final adjustments and then (53 - 64) to align completely


% DO NOT CHANGE THE FOLLOWING LINE
robot.p_in0 = nan(size(robot.p_in3));
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function RunSimulation(robot,mythruster,params)

%
% INITIALIZATION
%

% Define a global variable.
global thruster
thruster = mythruster;
% Define the geometry of the wall.
wall = GetGeometryOfWall;
% Create figure.
world = CreateFigure(robot,thruster,wall,params.teamname);
% Define time at which to start the simulation.
t = 0;
% Define time step.
dt = 2e-2;
% Define time limit.
tmax = 60;
% Define initial conditions
o_3in0 = [-6;-7;-2];
theta = [pi/2;0;pi/2];
v_03in0 = zeros(3,1);
w_03in3 = zeros(3,1);

%
% SIMULATION LOOP
%

% Either start making a movie or start storing actions.
if (params.makemovie)
    load(params.action_filename);
    myV = VideoWriter(params.movie_filename);
    myV.Quality = 100;
    open(myV);
else
    thrusterRecord = [];
end
% This variable stores the time the robot is close to the target. It must
% be "close" for 1 second before success.
tClose = 0;
% Loop until 'done' is true.
done = false;
while (~done)
    
    % Either store or retrieve the set of thrusters that are "on".
    curthruster = thruster;
    if (params.makemovie)
        [thrusterRecord,curthruster] = RetrieveThruster(thrusterRecord,curthruster);
        thruster = curthruster;
    else
        thrusterRecord = StoreThruster(thrusterRecord,curthruster);
    end
    % Solve ODEs to find position and orientation after time step.
    [t,o_3in0,theta,v_03in0,w_03in3] = Simulate(t,o_3in0,theta,v_03in0,w_03in3,robot,curthruster,dt);
    % Compute position and orientation of 3 in 0 and of 0 in 3
    R_3in0 = GetR_3in0(theta);
    R_0in3 = R_3in0';
    o_0in3 = -R_3in0'*o_3in0;
    % Transform stuff from frame 3 to frame 0
    for i=1:size(robot.p_in3,2)
        robot.p_in0(:,i) = o_3in0 + R_3in0*robot.p_in3(:,i);
    end
    for i=1:length(thruster)
        thruster(i).p_in0 = o_3in0+R_3in0*thruster(i).p_in3;
    end
    for i=1:size(wall.p_in0,2)
        wall.p_in3(:,i) = o_0in3 + R_0in3*wall.p_in0(:,i);
    end
    % Check if robot is close to the target.
    if (CheckCompletion(o_3in0,R_3in0))
        tClose = tClose + dt;
        if (tClose>1)
            done = true;
            world = DisplaySuccess(world);
        end
    else
        tClose = 0;
    end
    % Check if robot has collided with the wall.
    if (CheckCollision(robot.p_in0,robot.faces,wall)||t>tmax)
        done = true;
        world = DisplayFailure(world);
    end
    % Update the figure.
    world = UpdateFigure(world,robot,thruster,wall,o_3in0,R_3in0,o_0in3,R_0in3,t,tmax,tClose);
    % If making a movie, store the current figure as a frame.
    if (params.makemovie)
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    
end
% Either close the movie or save the record of thruster on/off's.
if (params.makemovie)
    close(myV);
else
    save(params.action_filename,'thrusterRecord');
end

function thrusterRecord = StoreThruster(thrusterRecord,thruster)
for i=1:length(thruster)
    rec(i,1) = thruster(i).on;
end
thrusterRecord(:,end+1) = rec;

function [thrusterRecord,thruster] = RetrieveThruster(thrusterRecord,thruster)
for i=1:length(thruster)
     thruster(i).on = thrusterRecord(i,1);
end
thrusterRecord = thrusterRecord(:,2:end);

function res = CheckCollision(p,f,wall)
p = p - repmat(wall.center,1,size(p,2));
for i=1:size(f,1)
    pcur = p(:,f(i,:));
    pcur = [pcur pcur(:,1)];
    for j=1:3
        p1 = pcur(:,j);
        p2 = pcur(:,j+1);
        if (p1(1)*p2(1)<0)
            s = p1(1)/(p1(1)-p2(1));
            q = (1-s)*p1+s*p2;
            if (abs(q(2))>wall.dw||abs(q(3))>wall.dw)
                res = 1;
                return;
            end
        end
    end
end
res = 0;

function res = CheckCompletion(o_3in0,R_3in0)
[mu,a] = GetExpCoords(R_3in0);
if ((mu<0.5236)&&(norm(o_3in0)<0.5))
    res = 1;
else
    res = 0;
end

function [mu,a] = GetExpCoords(R)
% If the equivalent angle is less than tol, we treat it as zero.
tol = 1e-2;
% Find the equivalent angle.
mu = acos((trace(R)-1)/2);
if (mu<tol)
    mu = 0;
    a = [1;0;0];
else
    % Find the equivalent axis.
    a = (1/(2*sin(mu)))*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
end

function [o_3in0dot,thetadot,v_03in0dot,w_03in3dot] = GetRates(o_3in0,theta,v_03in0,w_03in3,m,J_in3,thruster)
o_3in0dot = v_03in0;
thetadot = GetThetaDot(theta,w_03in3);
[f_in0,tau_in3] = GetForceAndMoment(thruster,theta);
v_03in0dot = (1/m)*f_in0;
w_03in3dot = inv(J_in3)*(tau_in3-wedge(w_03in3)*J_in3*w_03in3);

function R = rotX(h)
R = [1 0 0; 0 cos(h) -sin(h); 0 sin(h) cos(h)];
function R = rotY(h)
R = [cos(h) 0 sin(h); 0 1 0; -sin(h) 0 cos(h)];
function R = rotZ(h)
R = [cos(h) -sin(h) 0; sin(h) cos(h) 0; 0 0 1];
function R_3in0 = GetR_3in0(theta)
R_3in0 = rotX(theta(1))*rotY(theta(2))*rotZ(theta(3));
function wHat = wedge(w)
wHat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
function [f_in0,tau_in3] = GetForceAndMoment(thruster,theta)
R_3in0 = GetR_3in0(theta);
f_in0 = zeros(3,1);
tau_in3 = zeros(3,1);
for i=1:length(thruster)
    if (thruster(i).on)
        f_in0 = f_in0 + R_3in0*thruster(i).f_in3;
        tau_in3 = tau_in3 + wedge(thruster(i).p_in3)*thruster(i).f_in3;
    end
end
function thetadot = GetThetaDot(theta,w_03in1)
c3 = cos(theta(3));
s3 = sin(theta(3));
c2 = cos(theta(2));
s2 = sin(theta(2));
thetadot = [c3/c2 -s3/c2 0; s3 c3 0; -s2*c3/c2 s2*s3/c2 1]*w_03in1;

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

function wall = GetGeometryOfWall
wall.center = [-3;-3;-0.5];
wall.dy = 12;
wall.dz = 6;
wall.dw = 1.5;
wall.p_in0 = [zeros(1,8);
              -wall.dy wall.dy wall.dy -wall.dy -wall.dw wall.dw wall.dw -wall.dw;
              -wall.dz -wall.dz wall.dz wall.dz -wall.dw -wall.dw wall.dw wall.dw];
wall.p_in0 = repmat(wall.center,1,size(wall.p_in0,2))+wall.p_in0;
wall.p_in3 = nan(size(wall.p_in0));
wall.faces = [1 2 5;
              5 2 6;
              6 2 3;
              6 3 7;
              7 3 8;
              8 3 4;
              1 8 4;
              1 5 8];

function robot = DrawRobot(robot,p,f,alpha)
if isempty(robot)
    robot = patch('Vertices',p','Faces',f,'FaceColor','y',...
                  'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                  'backfacelighting','reverselit','AmbientStrength',0.6);
else
    set(robot,'vertices',p');
end

function wall = DrawWall(wall,p,f,alpha)
if isempty(wall)
    wall = patch('Vertices',p','Faces',f,'FaceColor','r',...
                  'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                  'backfacelighting','reverselit','AmbientStrength',0.6);
else
    set(wall,'vertices',p');
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

function world = CreateFigure(robot,thruster,wall,teamname)
% - clear the current figure
clf;
% - placeholders (these things have the wrong value)
o_0in0 = zeros(3,1);
R_0in0 = eye(3);
o_3in0 = zeros(3,1);
R_3in0 = eye(3);
o_1in3 = zeros(3,1);
R_1in3 = eye(3);
o_0in3 = zeros(3,1);
R_0in3 = eye(3);
% - text (it's important this is in the back, so you can rotate the view
%         and other stuff!)
axes('position',[0 0 1 1]);
axis([0 1 0 1]);
hold on;
axis off;
fs = 10;
world.text.time=text(0.05,0.1,sprintf('t = %6.2f / %6.2f\n',0,0),'fontsize',fs,'verticalalignment','top','fontname','monaco');
world.text.result=text(0.6,0.04,'','fontsize',36);
world.text.teamname=text(0.05,0.04,teamname,'fontsize',fs,'verticalalignment','top','fontweight','bold');
% - view from frame 0
view0axis = axes('position',[0.05 -0.05 .9 1.15]);
title('view: frame 0                                                 ');
set(gcf,'renderer','opengl');
axis equal;
axis([-6+wall.center(1) 6+wall.center(1) -6+wall.center(2) 6+wall.center(2) -3+wall.center(3) 3+wall.center(3)]);
axis manual;
hold on;
view([90-37.5,20]);
box on;
set(gca,'projection','perspective');
set(gca,'clipping','on','clippingstyle','3dbox');
world.view0.wall = DrawWall([],wall.p_in0,wall.faces,0.9);
world.view0.robot = DrawRobot([],robot.p_in0,robot.faces,0.9);
world.view0.frame0 = DrawFrame([],o_0in0,R_0in0);
world.view0.frame3 = DrawFrame([],o_3in0,R_3in0);
for i=1:length(thruster)
    world.view0.thruster(i) = DrawThruster([],thruster(i).p_in0,'k');
end
lighting gouraud
world.view0.light = light('position',o_3in0','style','local');
% - view from frame 3
axes('position',[0.53 0.73 0.445 0.23]);
title('view: frame 3');
axis off;
axis([0 1 0 1]);
axis manual;
hold on;
box on;
rectangle('position',[0 0 1 1],'facecolor','w','edgecolor','k');
axes('position',[0.655 0.78 0.2 0.2]);
axis equal;
axis([-.7 1e3 -2.2 2.2 -0.2 1.5]);
axis manual;
hold on;
box on;
world.view3.wall = DrawWall([],wall.p_in3,wall.faces,1);
world.view3.robot = DrawRobot([],robot.p_in3,robot.faces,1);
world.view3.frame0 = DrawFrame([],o_0in3,R_0in3);
world.view3.frame3 = DrawFrame([],o_1in3,R_1in3);
for i=1:length(thruster)
    world.view3.thruster(i) = DrawThruster([],thruster(i).p_in3,'k');
end
% - make the figure respond to key commands
set(gcf,'KeyPressFcn',@onkeypress);
lighting gouraud
world.view3.light = light('position',[0 0 1],'style','local');
set(gca,'projection','perspective');
campos(1*[-2;0;0.9]);
camtarget([1e3;0;0]);
camva(60);
axis off;
set(gca,'clipping','on','clippingstyle','rectangle');


function world = UpdateFigure(world,robot,thruster,wall,o_3in0,R_3in0,o_0in3,R_0in3,t,tmax,tClose)
world.view0.robot = DrawRobot(world.view0.robot,robot.p_in0,robot.faces);
world.view0.frame3 = DrawFrame(world.view0.frame3,o_3in0,R_3in0);
world.view3.robot = DrawRobot(world.view3.robot,robot.p_in3,robot.faces);
world.view3.frame0 = DrawFrame(world.view3.frame0,o_0in3,R_0in3);
world.view3.wall = DrawWall(world.view3.wall,wall.p_in3,wall.faces);
set(world.view0.light,'position',(o_3in0+R_3in0*[0;0;1])');
for i=1:length(thruster)
    if (thruster(i).on)
        world.view0.thruster(i) = DrawThruster(world.view0.thruster(i),thruster(i).p_in0,[0.9 0.5 0]);
        world.view3.thruster(i) = DrawThruster(world.view3.thruster(i),thruster(i).p_in3,[0.9 0.5 0]);
    else
        world.view0.thruster(i) = DrawThruster(world.view0.thruster(i),thruster(i).p_in0,'k');
        world.view3.thruster(i) = DrawThruster(world.view3.thruster(i),thruster(i).p_in3,'k');
    end
end
set(world.text.time,'string',sprintf('t = %6.2f / %6.2f\n',t,tmax));
if (tClose>0)
    set(world.view0.robot,'FaceColor','c');
    set(world.view3.robot,'FaceColor','c');
else
    set(world.view0.robot,'FaceColor','y');
    set(world.view3.robot,'FaceColor','y');
end
drawnow

function onkeypress(src,event)
global thruster
for i=1:length(thruster)
    if (i==10)
        if event.Character == '0'
            thruster(10).on = ~thruster(10).on;
        end
    elseif event.Character == char(48+i)
        thruster(i).on = ~thruster(i).on;
    end
end

function world = DisplaySuccess(world)
set(world.text.result,'string','SUCCESS!!!');

function world = DisplayFailure(world)
set(world.text.result,'string','FAILURE!!!');
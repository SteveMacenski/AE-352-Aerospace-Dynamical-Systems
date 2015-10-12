function hw5

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN CHANGE
%

% - This line says your name.
params.teamname = 'My Team (Chris Lorenz & Steven Macenski)';
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
%
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

function [wedge] = wedge(vector)
wedge = [0 -vector(3) vector(2);
         vector(3) 0 vector(1);
         -vector(2) vector(1) 0];

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MUST CHANGE
%

% - Mass and moment of inertia of SC
robot.sc.m = 24;
robot.sc.J_in3 = 24/12*[17 0 0;0 37 0;0 0 52];

% - Mass and moment of inertia of RW#4 
robot.rw4.m = 3;
robot.rw4.J_in4 = 3/12*[3.25 0 0;0 5 0;0 0 6.25];

% - Mass and moment of inertia of RW#5
robot.rw5.m = 3;
robot.rw5.J_in5 = 3/12*[6.25 0 0;0 3.25 0;0 0 5];

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN'T CHANGE
%

function RunSimulation(robot,params)

% Create figure.
world = CreateFigure(robot,params);
% Define time at which to start the simulation.
t = 0;
% Define time step.
dt = 2e-2;

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN CHANGE
%

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

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN'T CHANGE
%

% Either start making a movie or start storing actions.
global action done
action = [0;0];
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
        [actionRecord,curaction] = RetrieveAction(actionRecord);
        action = curaction;
    else
        curaction = action;
        actionRecord = StoreAction(actionRecord,curaction);
    end
    
    % Compute input torques applied by SC to each RW.
    u4 = curaction(1)*robot.rw4.tau;
    u5 = curaction(2)*robot.rw5.tau;
    
    % Solve ODEs.
    [t,o_3in0,theta,phi4,phi5,v_03in0,w_03in3,phi4dot,phi5dot] = ...
        Simulate(t,dt,...                               % <- time and time step
                 o_3in0,theta,phi4,phi5,...             % <- configuration
                 v_03in0,w_03in3,phi4dot,phi5dot,...    % <- velocity
                 u4,u5,...                              % <- applied torques
                 robot);                                % <- parameters that describe the SC and RWs
    
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MUST CHANGE
%
    
    % Compute orientation of 3 in 0
    %  (hint: you have theta - assume XYZ Euler Angle sequence, as usual)
    R_3in0 = [cos(theta(2))*cos(theta(3)) -cos(theta(2))*sin(theta(3)) sin(theta(2)); ...
        sin(theta(1))*sin(theta(2))*cos(theta(3))+cos(theta(1))*sin(theta(3))  (-1)*sin(theta(1))*sin(theta(2))*sin(theta(3))+cos(theta(1))*cos(theta(3)) (-1)*sin(theta(1))*cos(theta(2)); ...
        -cos(theta(1))*sin(theta(2))*cos(theta(3))+sin(theta(1))*sin(theta(3)) cos(theta(1))*sin(theta(2))*sin(theta(3))+sin(theta(1))*cos(theta(3)) cos(theta(1))*cos(theta(2))];
    
    % Compute position and orientation of 4 and 5 in 0
    %  (hint: you have o_3in0, R_3in0, phi4, phi5,
    %                  robot.o_4in3, robot.o_5in3)
    o_4in0 = o_3in0 + R_3in0*robot.o_4in3;
    
    o_5in0 = o_3in0 + R_3in0*robot.o_5in3;
    
    R_4in3 = [cos(phi4) 0 sin(phi4);...
              0         1       0 ; 
              -sin(phi4) 0 cos(phi4)];
    R_4in0 = R_3in0*R_4in3;      
          
    R_5in3 = [cos(phi5) -sin(phi5) 0;...
              sin(phi5) cos(phi5) 0;
              0 0 1];
    R_5in0 = R_3in0*R_5in3;
    
    % Compute robot.sc.p_in0, robot.rw4.p_in0, robot.rw5.p_in0
    %  (hint: you have o_3in0, R_3in0, o_4in0, R_4in0, o_5in0, R_5in0,
    %                  robot.sc.p_in3, robot.rw4.p_in4, robot.rw5.p_in5)
    robot.sc.p_in0 = [o_3in0 o_3in0 o_3in0 o_3in0 o_3in0 o_3in0 o_3in0 o_3in0] + R_3in0*robot.sc.p_in3;
    robot.rw4.p_in0 = [o_4in0 o_4in0 o_4in0 o_4in0 o_4in0 o_4in0 o_4in0 o_4in0]+ R_4in0*robot.rw4.p_in4;
    robot.rw5.p_in0 = [o_5in0 o_5in0 o_5in0 o_5in0 o_5in0 o_5in0 o_5in0 o_5in0] + R_5in0*robot.rw5.p_in5;
    
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN'T CHANGE
%
    
    % Update the figure.
    world = UpdateFigure(world,robot,o_3in0,R_3in0,t,tmax);
    
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

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MUST CHANGE
%

function [o_3in0dot,thetadot,phi4dot,phi5dot,...             % <- velocity
          v_03in0dot,w_03in3dot,phi4dotdot,phi5dotdot] = ... % <- acceleration
             GetRates(o_3in0,theta,phi4,phi5,...             % <- configuration
                      v_03in0,w_03in3,phi4dot,phi5dot,...    % <- velocity
                      u4,u5,...                              % <- input torques
                      robot)                                 % <- parameters that describe the SC and RWs
m4 = robot.rw4.m; 
m5 = robot.rw5.m;
m = robot.sc.m;

t4 = 0; t5 = 0; s4 = 0; s5 = 0;

  R_3in0 = [cos(theta(2))*cos(theta(3)) -cos(theta(2))*sin(theta(3)) sin(theta(2)); ...
        sin(theta(1))*sin(theta(2))*cos(theta(3))+cos(theta(1))*sin(theta(3))  (-1)*sin(theta(1))*sin(theta(2))*sin(theta(3))+cos(theta(1))*cos(theta(3)) (-1)*sin(theta(1))*cos(theta(2)); ...
        -cos(theta(1))*sin(theta(2))*cos(theta(3))+sin(theta(1))*sin(theta(3)) cos(theta(1))*sin(theta(2))*sin(theta(3))+sin(theta(1))*cos(theta(3)) cos(theta(1))*cos(theta(2))];
    
    R_4in3 = [cos(phi4) 0 sin(phi4);...
              0         1       0 ; 
              -sin(phi4) 0 cos(phi4)];
    R_4in0 = R_3in0*R_4in3;      
          
    R_5in3 = [cos(phi5) -sin(phi5) 0;...
              sin(phi5) cos(phi5) 0;
              0 0 1];
    R_5in0 = R_3in0*R_5in3;
    
    t4 = [0;1;0];
    t5 = [0;0;1];
    
    s4 = [1 0; 0 0; 0 1];
    s5 = [1 0; 0 1; 0 0];
      w_34in4 = zeros(3,1);
      w_04in4 = zeros(3,1);
      w_35in5 = zeros(3,1);
      w_05in5 = zeros(3,1);

 h = [zeros(3,1);
      -m4*R_3in0*wedge(w_03in3)*wedge(w_03in3)*robot.o_4in3;
      -m5*R_3in0*wedge(w_03in3)*wedge(w_03in3)*robot.o_5in3;
      -wedge(w_03in3)*robot.sc.J_in3*w_03in3 - t4*u4 - t5*u5;
      -robot.rw4.J_in4*(R_4in3*wedge(w_34in4))'*w_03in3 - wedge(w_04in4)*robot.rw4.J_in4*w_04in4 + R_4in3'*t4*u4;
      -robot.rw5.J_in5*(R_5in3*wedge(w_35in5))'*w_03in3 - wedge(w_05in5)*robot.rw5.J_in5*w_05in5 + R_5in3'*t5*u5];
  
  %size(h)

  
  F =   [m*eye(3) zeros(3) zeros(3,1) zeros(3,1) R_3in0 R_3in0 zeros(3,2) zeros(3,2);...
        m4*eye(3) -m4*R_3in0*wedge(robot.o_4in3) zeros(3,1) zeros(3,1) -R_3in0 zeros(3) zeros(3,2) zeros(3,2);
        m5*eye(3) -m5*R_3in0*wedge(robot.o_5in3) zeros(3,1) zeros(3,1) zeros(3) -R_3in0 zeros(3,2) zeros(3,2);
        zeros(3) robot.sc.J_in3 zeros(3,1) zeros(3,1) wedge(robot.sc.p_in3) wedge(robot.sc.p_in3) s4 s5;
        zeros(3) robot.rw4.J_in4*R_4in3' robot.rw4.J_in4*t4 zeros(3,1) -wedge(robot.rw4.p_in4)*R_4in3' zeros(3) -R_4in3'*s4 zeros(3,2);
        zeros(3) robot.rw5.J_in5*R_5in3' zeros(3,1) robot.rw5.J_in5*t5 zeros(3,3) -wedge(robot.rw5.p_in5)*R_5in3' zeros(3,2) -R_5in3'*s5];
  %size(F)
  x = F\h              
                  
o_3in0dot = v_03in0;
thetadot = [phi4dot; phi5dot; 0]; %so so terribly wrong, correct this TODO
phi4dotdot = x(7:9);
phi5dotdot = x(10:12);
v_03in0dot = x(1:3);
w_03in3dot = x(4:6);
phi4dot = phi4dot;
phi5dot = phi5dot;

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN'T CHANGE
%

function actionRecord = StoreAction(actionRecord,action)
actionRecord(:,end+1) = action;

function [actionRecord,action] = RetrieveAction(actionRecord)
action = actionRecord(:,1);
actionRecord = actionRecord(:,2:end);

function [t,o_3in0,theta,phi4,phi5,v_03in0,w_03in3,phi4dot,phi5dot] = ...
            Simulate(t,dt,...                               % <- time and time step
                     o_3in0,theta,phi4,phi5,...             % <- configuration
                     v_03in0,w_03in3,phi4dot,phi5dot,...    % <- velocity
                     u4,u5,...                              % <- input torques
                     robot)                                 % <- parameters that describe the SC and RWs
[t,x] = ode45(@(t,x) GetXDot(t,x,u4,u5,robot),[t t+dt],[o_3in0;theta;phi4;phi5;v_03in0;w_03in3;phi4dot;phi5dot]);
x = x';
t = t(end);
[o_3in0,theta,phi4,phi5,v_03in0,w_03in3,phi4dot,phi5dot] = XToState(x);

function xdot = GetXDot(t,x,u4,u5,robot)
% Unpack the state.
[o_3in0,theta,phi4,phi5,v_03in0,w_03in3,phi4dot,phi5dot] = XToState(x);
% Get time derivatives.
[o_3in0dot,thetadot,phi4dot,phi5dot,...             % <- velocity
 v_03in0dot,w_03in3dot,phi4dotdot,phi5dotdot] = ... % <- acceleration
    GetRates(o_3in0,theta,phi4,phi5,...             % <- configuration
             v_03in0,w_03in3,phi4dot,phi5dot,...    % <- velocity
             u4,u5,...                              % <- input torques
             robot);                                % <- parameters that describe the SC and RWs
% Pack time derivatives.

xdot = [o_3in0dot;thetadot;phi4dot;phi5dot;v_03in0dot;w_03in3dot;phi4dotdot;phi5dotdot];

function [o_3in0,theta,phi4,phi5,v_03in0,w_03in3,phi4dot,phi5dot] = XToState(x)
o_3in0 = x(1:3,end);
theta = x(4:6,end);
phi4 = x(7,end);
phi5 = x(8,end);
v_03in0 = x(9:11,end);
w_03in3 = x(12:14,end);
phi4dot = x(15,end);
phi5dot = x(16,end);

function robotfig = DrawRobot(robotfig,robot,o_3in0,R_3in0,alpha)
ered=[1,0.6,0];
egrey=[0.4745,0.6471,0.9098];
if isempty(robotfig)
    p = o_3in0+R_3in0*robot.rw4.ppin_in3;
    robotfig.rw4pin = line(p(1),p(2),p(3),'color','k','marker','.','markersize',24);
    p = o_3in0+R_3in0*robot.rw5.ppin_in3;
    robotfig.rw5pin = line(p(1),p(2),p(3),'color','k','marker','.','markersize',24);
    robotfig.sc = patch('Vertices',robot.sc.p_in0','Faces',robot.faces,'FaceColor',ered,...
                        'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                        'backfacelighting','reverselit','AmbientStrength',0.6);
    robotfig.rw4 = patch('Vertices',robot.rw4.p_in0','Faces',robot.faces,'FaceColor',egrey,...
                         'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                         'backfacelighting','reverselit','AmbientStrength',0.6);
    robotfig.rw5 = patch('Vertices',robot.rw5.p_in0','Faces',robot.faces,'FaceColor',egrey,...
                         'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                         'backfacelighting','reverselit','AmbientStrength',0.6);
else
    set(robotfig.sc,'vertices',robot.sc.p_in0');
    set(robotfig.rw4,'vertices',robot.rw4.p_in0');
    set(robotfig.rw5,'vertices',robot.rw5.p_in0');
    p = o_3in0+R_3in0*robot.rw4.ppin_in3;
    set(robotfig.rw4pin,'xdata',p(1),'ydata',p(2),'zdata',p(3));
    p = o_3in0+R_3in0*robot.rw5.ppin_in3;
    set(robotfig.rw5pin,'xdata',p(1),'ydata',p(2),'zdata',p(3));
end

function frame = DrawFrame(frame,o,R)
p = [o repmat(o,1,3)+R];
if isempty(frame)
    frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',4);
    frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',4);
    frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',4);
else
    set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
    set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
    set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));
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
world.view0.frame3 = DrawFrame([],o_3in0,R_3in0);
lighting gouraud
world.view0.light = light('position',o_3in0','style','local');
% - make the figure respond to key commands
if (params.usekeypad)
    set(gcf,'KeyPressFcn',@onkeypress_keypad);
else
    set(gcf,'KeyPressFcn',@onkeypress_nokeypad);
end

function world = UpdateFigure(world,robot,o_3in0,R_3in0,t,tmax)
world.view0.robot = DrawRobot(world.view0.robot,robot,o_3in0,R_3in0);
world.view0.frame3 = DrawFrame(world.view0.frame3,o_3in0,R_3in0);
set(world.view0.light,'position',(o_3in0+R_3in0*[0;0;1])');
set(world.text.time,'string',sprintf('t = %6.2f / %6.2f\n',t,tmax));
drawnow

function onkeypress_nokeypad(src,event)
global action done
if event.Character == '4'
    action(1,1) = 1;
elseif event.Character == 'r'
    action(1,1) = 0;
elseif event.Character == 'f'
    action(1,1) = -1;
elseif event.Character == '5'
    action(2,1) = 1;
elseif event.Character == 't'
    action(2,1) = 0;
elseif event.Character == 'g'
    action(2,1) = -1;
elseif event.Character == 'q'
    done = true;
end

function onkeypress_keypad(src,event)
global action done
if event.Character == '7'
    action(1,1) = 1;
elseif event.Character == '4'
    action(1,1) = 0;
elseif event.Character == '1'
    action(1,1) = -1;
elseif event.Character == '8'
    action(2,1) = 1;
elseif event.Character == '5'
    action(2,1) = 0;
elseif event.Character == '2'
    action(2,1) = -1;
elseif event.Character == 'q'
    done = true;
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
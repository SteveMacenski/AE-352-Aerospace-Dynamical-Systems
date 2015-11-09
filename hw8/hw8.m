function hw8

%%%%
%
% In order to get your walker to walk, you'll want to play with the
% following parameters:
%
%  in GetGeometryOfRobot...
%
%   robot.k             coefficient of viscous friction at revolute joint
%   robot.slopeangle    angle of slope down which the walker walks
%
%  in RunSimulation...
%
%   phi                 initial roll angle
%   
% What the walker does will change a lot depending on your choice of these
% three parameters.
%
%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN CHANGE
%

% - This line says your name.
params.teamname = 'My Team (My Name and My Other Name)';
% - This line says the file to record your movie.
params.movie_filename = 'movie.mp4';
% - This line says the file to record your snapshot.
params.snapshot_filename = 'snapshot.pdf';
% - This line says whether or not you want to record a movie --- change it
%   from "false" to "true" and you will record a movie.
params.makemovie = false;
% - This line says whether or not you want to take a snapshot --- change it
%   from "false" to "true" and you will create a PDF of the figure after
%   the simulation is over.
params.makesnapshot = true;

% The keyboard interface is as follows:
%
%   'q'         causes the simulation to quit (gracefully, saving the
%               movie and/or the snapshot)
%

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN'T CHANGE
%

% Define the geometry and mass properties of the robot.
robot = GetGeometryOfRobot;

% Run the simulation.
RunSimulation(robot,params);

end

function robot = GetGeometryOfRobot
robot=[];

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN CHANGE
%

% - coefficient of friction at the revolute joint
robot.k = .01;

% - angle of slope (radians)
robot.slopeangle = 0.3;

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN'T CHANGE
%

% - force of gravity vector
robot.g_in0 = -9.81*[0;-sin(robot.slopeangle);cos(robot.slopeangle)];

% - colors
robot.colors.uiuc_orange=[1,0.6,0];
robot.colors.uiuc_blue=[0.4745,0.6471,0.9098];
robot.colors.uiuc_brown=[0.4,0.2,0];

% - radius of outer sphere
robot.r = 1;

% - height of spherical cap
capheight = .25;

% - mass of half-cap
m = 0.0899935;

% - CM of half-cap
pcm = [0.227701;0;0.164773];

% - J of half-cap with respect to CM
J = [0.00704179 0 -0.000234689;
     0 0.00237584 0;
     -0.000234689 0 0.00878195];

% - moment of inertia matrix of left foot (1) and right foot (2)
robot.J_Lin1 = J;
robot.J_Rin2 = J;

% - mass of left foot (1) and right foot (2)
robot.mL = m;
robot.mR = m;

% - template for spherical cap
[p,f]=GetSphericalCap(robot.r,capheight,3);

% - faces and colors for both feet
robot.fL = f;
robot.fR = f;
robot.cL = repmat(robot.colors.uiuc_blue,size(robot.fL,1),1);
robot.cR = repmat(robot.colors.uiuc_orange,size(robot.fR,1),1);

% - vertices for both feet
robot.pL_in1 = [-p(1,:); -p(2,:); p(3,:)]-repmat([-pcm(1);-pcm(2);pcm(3)],1,size(p,2));
robot.pR_in2 = p-repmat(pcm,1,size(p,2));

% - length parameters
robot.a = robot.r-pcm(3);
robot.b = pcm(1);

% - location of revolute joint
robot.p_12in1 = [robot.b;0;robot.a];
robot.p_12in2 = [-robot.b;0;robot.a];

% - vertices, faces, and colors for joint brackets
A = 0.2*robot.b;
B = 3*A;
C = capheight-pcm(3);
robot.pJointR_in2 = [-A/2 A/2 A/2 -robot.b -robot.b -A/2 -A/2 A/2 A/2 -robot.b -robot.b -A/2;
                -B/2 -B/2 -B/2 -B/2 -B/2 -B/2 B/2 B/2 B/2 B/2 B/2 B/2;
                C C robot.a+(A/2) robot.a+(A/2) robot.a-(A/2) robot.a-(A/2) C C robot.a+(A/2) robot.a+(A/2) robot.a-(A/2) robot.a-(A/2)];
robot.fJointR = [1 2 6;
            2 3 6;
            3 4 6;
            4 5 6;
            8 7 12;
            8 12 9;
            12 11 10;
            12 10 9;
            2 8 9;
            2 9 3;
            3 9 10;
            3 10 4;
            1 7 2;
            7 8 2;
            7 1 6;
            7 6 12;
            12 6 11;
            11 6 5;
            11 5 4;
            11 4 10];
robot.pJointL_in1 = [-robot.pJointR_in2(1,:); -robot.pJointR_in2(2,:); robot.pJointR_in2(3,:)];
robot.fJointL = robot.fJointR;
robot.cJointL = repmat(robot.colors.uiuc_blue,size(robot.fJointL,1),1);
robot.cJointR = repmat(robot.colors.uiuc_orange,size(robot.fJointR,1),1);

end

function RunSimulation(robot,params)

% Create empty figure.
world = [];
% Define time at which to start the simulation.
t = 0;
% Define time step.
dt = 4e-2;

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

% Define initial conditions (be careful changing anything except "phi").

% - initial roll angle (radians)
phi = 0.2;

% - initial orientation in terms of roll angle
thetaL = [0;phi;0];
thetaR = [0;phi;0];

% - initial position in terms of roll angle
o_1in0 = [0;0;robot.r]+R_ZYX(thetaL)*[-robot.b;0;-robot.a];
o_2in0 = [0;0;robot.r]+R_ZYX(thetaR)*[robot.b;0;-robot.a];

% - linear and angular velocity
v_01in0 = zeros(3,1);
w_01in1 = zeros(3,1);
v_02in0 = zeros(3,1);
w_02in2 = zeros(3,1);

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN'T CHANGE
%

% - on which foot
if (phi>0)
    whichfoot = 2;
else
    whichfoot = 1;
end

% Set a flag to say when we are done simulating.
global done
done = false;

% If making a movie, start making it.
if (params.makemovie)
    myV = VideoWriter(params.movie_filename,'MPEG-4');
    myV.Quality = 100;
    myV.FrameRate = 25;
    open(myV);
end

% Loop until break.
while (1)
    
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MUST CHANGE
%
    
    %
    % Compute: (orientations)
    %   
    %   Assume the use of ZYX Euler Angles (**not** XYZ!!!!).
    %
    %       thetaL is a 3x1 matrix of ZYX Euler Angles that describe the
    %       orientation of frame 1 (so thetaL(1) is the angle of rotation
    %       about z0, and so forth)
    %
    %       thetaR is a 3x1 matrix of the ZYX E.A.'s for frame 2
    %
    R_1in0 = eye(3);
    R_2in0 = eye(3);
    
    %
    % Compute: (coordinate transformations)
    %
    robot.pL_in0 = robot.pL_in1;
    robot.pR_in0 = robot.pR_in2;
    robot.pJointL_in0 = robot.pJointL_in1;
    robot.pJointR_in0 = robot.pJointR_in2;
    
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN'T CHANGE
%
    
    % Update the figure.
    world = UpdateFigure(world,robot,params,o_1in0,R_1in0,o_2in0,R_2in0,t,tmax);
    
    % If making a movie, store the current figure as a frame.
    if (params.makemovie)
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    
    % Stop if time has reached its maximum.
    if ((t>=tmax)||done)
        break;
    end
    
    % Solve ODEs.
    [t,o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2,whichfoot] = ...
            Simulate(t,dt,...
                     o_1in0,o_2in0,thetaL,thetaR,...
                     v_01in0,v_02in0,w_01in1,w_02in2,...
                     whichfoot,...
                     robot);
    
end
% If making a movie, close it.
if (params.makemovie)
    for i=1:myV.FrameRate
        frame = getframe(gcf);
        writeVideo(myV,frame);
    end
    close(myV);
end

% If making a snapshot, make it.
if (params.makesnapshot)
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    print(gcf,'-dpdf',params.snapshot_filename);
end

end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MUST CHANGE
%

%
% This function computes rates of change assuming the right foot is in
% contact with the ground.
%
function [o_1in0dot,o_2in0dot,thetaLdot,thetaRdot,v_01in0dot,v_02in0dot,w_01in1dot,w_02in2dot] = ...
    GetRatesR(o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2,robot)

% - position and orientation
o_1in0dot = zeros(3,1);
o_2in0dot = zeros(3,1);
thetaLdot = zeros(3,1);
thetaRdot = zeros(3,1);

% - linear and angular velocity
v_01in0dot = zeros(3,1);
w_01in1dot = zeros(3,1);
v_02in0dot = zeros(3,1);
w_02in2dot = zeros(3,1);

end

%
% This function computes rates of change assuming the left foot is in
% contact with the ground.
%
function [o_1in0dot,o_2in0dot,thetaLdot,thetaRdot,v_01in0dot,v_02in0dot,w_01in1dot,w_02in2dot] = ...
    GetRatesL(o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2,robot)

% - position and orientation
o_1in0dot = zeros(3,1);
o_2in0dot = zeros(3,1);
thetaLdot = zeros(3,1);
thetaRdot = zeros(3,1);

% - linear and angular velocity
v_01in0dot = zeros(3,1);
w_01in1dot = zeros(3,1);
v_02in0dot = zeros(3,1);
w_02in2dot = zeros(3,1);

end

%
% This function computes the linear and angular velocities after the impact
% that occurs when switching from the left foot to the right foot.
%
function [v_01in0,v_02in0,w_01in1,w_02in2] = ...
            SwitchFromLToR(o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2,robot)


end

%
% This function computes the linear and angular velocities after the impact
% that occurs when switching from the right foot to the left foot.
%
function [v_01in0,v_02in0,w_01in1,w_02in2] = ...
            SwitchFromRToL(o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2,robot)


end


%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% CAN'T CHANGE
%

function [t,o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2,whichfoot] = ...
            Simulate(t,dt,...
                     o_1in0,o_2in0,thetaL,thetaR,...
                     v_01in0,v_02in0,w_01in1,w_02in2,...
                     whichfoot,...
                     robot)
tFinal = t+dt;
while (t<tFinal)
    tEvent = [];
    [t,x,tEvent] = ode45(@(t,x) GetXDot(t,x,robot,whichfoot),[t t+dt],...
       [o_1in0;o_2in0;thetaL;thetaR;v_01in0;v_02in0;w_01in1;w_02in2],...
       odeset('Events',@(t,x) FindSwitch(t,x,robot,whichfoot),'RelTol',1e-6,'AbsTol',1e-9));
    x = x';
    t = t(end);
    [o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2] = XToState(x);
    if (isempty(tEvent))
        % Stayed on the same foot the whole time.
        break;
    else
        % Arrived at a switch from one foot to the other foot.
        dt = tFinal-t;
        if (whichfoot==1)
            % The switch was from left to right.
            whichfoot = 2;
            [v_01in0,v_02in0,w_01in1,w_02in2] = ...
                SwitchFromLToR(o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2,robot);
        elseif (whichfoot==2)
            % The switch was from right to left.
            whichfoot = 1;
            [v_01in0,v_02in0,w_01in1,w_02in2] = ...
                SwitchFromRToL(o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2,robot);
        else
            error('whichfoot=%d, should be 1 or 2',whichfoot);
        end
    end
end
end

% A function returning a value that allows ode45 to detect when there is a
% switch from one foot to the other foot.
%
% - when "value" is positive, the right foot is in contact
% - when "value" is negative, the left foot is in contact
%
function [value,isterminal,direction] = FindSwitch(t,x,robot,whichfoot)
% - unpack the state
[o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2] = XToState(x);
% - say we want ode45 to terminate when it finds a zero
isterminal=1;
% - check which foot is currently in contact
if (whichfoot==1)
    % - the left foot is in contact; compute "value"
    R_1in0 = R_ZYX(thetaL);
    p_01in1 = [robot.b;0;(robot.a)]+R_1in0'*[0;0;-robot.r];
    value = p_01in1(1)-robot.b;
    % - say we want ode45 to detect when "value" crosses zero from below
    direction = 1;
elseif (whichfoot==2)
    % - the right foot is in contact; compute "value"
    R_2in0 = R_ZYX(thetaR);
    p_02in2 = [-robot.b;0;(robot.a)]+R_2in0'*[0;0;-robot.r];
    value = p_02in2(1)+robot.b;
    % - say we want ode45 to detect when "value" crosses zero from above
    direction = -1;
else
    error('whichfoot=%d should be either 1 or 2',whichfoot);
end
end

function xdot = GetXDot(t,x,robot,whichfoot)
% Unpack the state.
[o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2] = XToState(x);
% Get time derivatives.
if (whichfoot==1)
    [o_1in0dot,o_2in0dot,thetaLdot,thetaRdot,v_01in0dot,v_02in0dot,w_01in1dot,w_02in2dot] = ...
        GetRatesL(o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2,robot);
elseif (whichfoot==2)
    [o_1in0dot,o_2in0dot,thetaLdot,thetaRdot,v_01in0dot,v_02in0dot,w_01in1dot,w_02in2dot] = ...
        GetRatesR(o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2,robot);
else
    error('whichfoot=%d should be either 1 or 2',whichfoot);
end
% Pack time derivatives.
xdot = [o_1in0dot;o_2in0dot;thetaLdot;thetaRdot;v_01in0dot;v_02in0dot;w_01in1dot;w_02in2dot];
end

function [o_1in0,o_2in0,thetaL,thetaR,v_01in0,v_02in0,w_01in1,w_02in2] = XToState(x)
o_1in0 = x(1:3,end);
o_2in0 = x(4:6,end);
thetaL = x(7:9,end);
thetaR = x(10:12,end);
v_01in0 = x(13:15,end);
v_02in0 = x(16:18,end);
w_01in1 = x(19:21,end);
w_02in2 = x(22:24,end);
end

function robotfig = DrawRobot(robotfig,robot,alpha)
if isempty(robotfig)
    % - feet
    robotfig.LFoot = patch('Vertices',robot.pL_in00','Faces',robot.fL,'FaceVertexCData',robot.cL,'FaceColor','flat',...
                          'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                          'backfacelighting','reverselit','AmbientStrength',0.6,...
                          'EdgeColor',0.9*robot.colors.uiuc_blue);
	robotfig.RFoot = patch('Vertices',robot.pR_in00','Faces',robot.fR,'FaceVertexCData',robot.cR,'FaceColor','flat',...
                          'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                          'backfacelighting','reverselit','AmbientStrength',0.6,...
                          'EdgeColor',0.9*robot.colors.uiuc_orange);
	% - joint brackets
    robotfig.LJoint = patch('Vertices',robot.pJointL_in00','Faces',robot.fJointL,'FaceVertexCData',robot.cJointL,'FaceColor','flat',...
                          'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                          'backfacelighting','reverselit','AmbientStrength',0.6,...
                          'EdgeColor',0.9*robot.colors.uiuc_blue);
	robotfig.RJoint = patch('Vertices',robot.pJointR_in00','Faces',robot.fJointR,'FaceVertexCData',robot.cJointR,'FaceColor','flat',...
                          'FaceAlpha',alpha,'EdgeAlpha',alpha,...
                          'backfacelighting','reverselit','AmbientStrength',0.6,...
                          'EdgeColor',0.9*robot.colors.uiuc_orange);
else
    set(robotfig.LFoot,'vertices',robot.pL_in00');
    set(robotfig.RFoot,'vertices',robot.pR_in00');
    set(robotfig.LJoint,'vertices',robot.pJointL_in00');
    set(robotfig.RJoint,'vertices',robot.pJointR_in00');
end
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
end

function world = CreateFigure(robot,params,o_1in0,R_1in0,o_2in0,R_2in0,t,tmax)
% Clear current figure.
clf;
% Text (important to do this first, so it's in the back).
axes('position',[0 0 1 1]);
axis([0 1 0 1]);
hold on;
axis off;
fs = 10;
world.text.label=text(0.15,0.95,'view: frame 0','fontweight','bold','fontsize',fs);
world.text.time=text(0.05,0.1,sprintf('t = %6.2f / %6.2f\n',t,tmax),'fontsize',fs,'verticalalignment','top','fontname','monaco');
world.text.teamname=text(0.05,0.04,params.teamname,'fontsize',fs,'verticalalignment','top','fontweight','bold');
% View from frame 0.
axes('position',[0.05 0.15 .9 .8]);
set(gcf,'renderer','opengl');
axis equal;
axis(10*[-1 1 -1 1 -1 1]);
axis manual;
hold on;
view([90-20,20]);
box on;
set(gca,'projection','perspective');
set(gca,'clipping','on','clippingstyle','3dbox');
% - transformations
R_00in0 = [1 0 0; 0 cos(robot.slopeangle) -sin(robot.slopeangle); 0 sin(robot.slopeangle) cos(robot.slopeangle)];
robot.pL_in00 = R_00in0'*robot.pL_in0;
robot.pR_in00 = R_00in0'*robot.pR_in0;
robot.pJointL_in00 = R_00in0'*robot.pJointL_in0;
robot.pJointR_in00 = R_00in0'*robot.pJointR_in0;
o_1in00 = R_00in0'*o_1in0;
o_2in00 = R_00in0'*o_2in0;
R_1in00 = R_00in0'*R_1in0;
R_2in00 = R_00in0'*R_2in0;
% - ground
L = 10;
W = 2;
A = L*sin(robot.slopeangle)^2;
B = L*cos(robot.slopeangle)^2;
D = L*sin(robot.slopeangle)*cos(robot.slopeangle);
pGround_in0 = [[W;-1+A;-D] [W;L-1;0] [W;-1;0] [-W;-1+A;-D] [-W;L-1;0] [-W;-1;0]];
pGround_in00 = R_00in0'*pGround_in0;
fGround = [1 2 3; 5 4 6; 1 4 5; 1 5 2; 4 1 3; 4 3 6; 2 5 6; 2 6 3];
cGround = repmat([1 1 0],size(fGround,1),1);
world.view0.ground = patch('Vertices',pGround_in00','Faces',fGround,'FaceVertexCData',cGround,'FaceColor','flat',...
                          'FaceAlpha',1,'EdgeAlpha',1,...
                          'backfacelighting','reverselit','AmbientStrength',0.9,...
                          'EdgeColor',0.5*[1 1 1]);
% - robot
world.view0.robot = DrawRobot([],robot,0.8);
% - frames
world.view0.frame0 = DrawFrame([],zeros(3,1),R_00in0');
world.view0.frame1 = DrawFrame([],o_1in00,R_1in00);
world.view0.frame2 = DrawFrame([],o_2in00,R_2in00);
% - axis limits
set(gca,'xlim',[-W W]);
set(gca,'ylim',cos(robot.slopeangle)*[-1 L-1]);
set(gca,'zlim',[-(L-1)*sin(robot.slopeangle) 2]);
% - lights
lighting gouraud
world.view0.light = light('position',[0;0;0.1]','style','local');
% - make the figure respond to key commands
set(gcf,'KeyPressFcn',@onkeypress_nokeypad);
end

function world = UpdateFigure(world,robot,params,o_1in0,R_1in0,o_2in0,R_2in0,t,tmax)
if (isempty(world))
    world = CreateFigure(robot,params,o_1in0,R_1in0,o_2in0,R_2in0,t,tmax);
else
    R_00in0 = [1 0 0; 0 cos(robot.slopeangle) -sin(robot.slopeangle); 0 sin(robot.slopeangle) cos(robot.slopeangle)];
    robot.pL_in00 = R_00in0'*robot.pL_in0;
    robot.pR_in00 = R_00in0'*robot.pR_in0;
    robot.pJointL_in00 = R_00in0'*robot.pJointL_in0;
    robot.pJointR_in00 = R_00in0'*robot.pJointR_in0;
    o_1in00 = R_00in0'*o_1in0;
    o_2in00 = R_00in0'*o_2in0;
    R_1in00 = R_00in0'*R_1in0;
    R_2in00 = R_00in0'*R_2in0;
    world.view0.robot = DrawRobot(world.view0.robot,robot);
    world.view0.frame1 = DrawFrame(world.view0.frame1,o_1in00,R_1in00);
    world.view0.frame2 = DrawFrame(world.view0.frame2,o_2in00,R_2in00);
    set(world.text.time,'string',sprintf('t = %6.2f / %6.2f\n',t,tmax));
end
drawnow
end

function onkeypress_nokeypad(src,event)
global done
if event.Character == 'q'
    done = true;
end
end

function i = FindPoint(q,p,tol)
% finds q in the list of points p to tol
% (assumes it exists exactly zero times or one time in p)
d = repmat(q,1,size(p,2))-p;
d = sqrt(sum(d.^2,1));
i = find(d<tol);
end

function e1231 = GetEdgeLabel(e12,e31)
if (e12+e31==3)
    e1231=1;
else
    e1231=0;
end
end

function eint = GetInternalEdgeLabels(e,flabel)
if (flabel==1)
    eint = [1 1 1];
else
    eint = [GetEdgeLabel(e(1),e(2)) GetEdgeLabel(e(2),e(3)) GetEdgeLabel(e(3),e(1))];
end
end

function [i12,p] = AddPoint(p,e,i1,i2,r,b,tol)

p12 = 0.5*(p(:,i1)+p(:,i2));
if (e==2)
    % the point is being created on an edge along the top curve
    p12(1:2,:) = b*p12(1:2,:)/norm(p12(1:2,:));
elseif (e==1)
    % the point is being created on an edge along the bottom
    p12 = r*p12/norm(p12);
else
    % the point is being created on an edge that is neither along the top
    % curve nor along the bottom (do nothing)
end

i12 = FindPoint(p12,p,tol);
if (isempty(i12))
    p(:,end+1) = p12;
    i12 = size(p,2);
end

end

function [p,f]=GetSphericalCap(r,a,niters)
% e
%  0 = "internal + flat" or "straight edge"
%  1 = bottom
%  2 = top curve
% f
%  0 = not bottom
%  1 = bottom

% - radius of half-disk at the top of the spherical cap
b = sqrt(a*(2*r-a));

p = [0 0 0 b; -b 0 b 0; -(r-a) -r -(r-a) -(r-a)];
f = [1 2 4; 2 3 4; 1 4 3; 1 3 2];
elabel = [1 1 2; 1 2 1; 2 2 0; 0 1 1];
flabel = [1 1 0 0];
tol = 1e-6;
for j=1:niters
    fnew = [];
    elabelnew = [];
    flabelnew = [];
    for i=1:size(f,1)

        % get points associated with current face
        i1 = f(i,1);
        i2 = f(i,2);
        i3 = f(i,3);
        
        % create new points in center of each edge
        [i12,p] = AddPoint(p,elabel(i,1),i1,i2,r,b,tol);
        [i23,p] = AddPoint(p,elabel(i,2),i2,i3,r,b,tol);
        [i31,p] = AddPoint(p,elabel(i,3),i3,i1,r,b,tol);
        
        % create four new faces
        fnew = [fnew; [i1 i12 i31; i2 i23 i12; i3 i31 i23; i12 i23 i31]];
        eint = GetInternalEdgeLabels(elabel(i,:),flabel(i));
        elabelnew = [elabelnew; [elabel(i,1) eint(3) elabel(i,3); elabel(i,2) eint(1) elabel(i,1); elabel(i,3) eint(2) elabel(i,2); eint]];
        flabelnew = [flabelnew; repmat(flabel(i),4,1)];
    
    end
    f = fnew;
    elabel = elabelnew;
    flabel = flabelnew;
end
p(3,:) = p(3,:) + r;
end

function R = RX(h)
R = [1 0 0;
     0 cos(h) -sin(h);
     0 sin(h) cos(h)];
end
 
function R = RY(h)
R = [cos(h) 0 sin(h);
     0 1 0;
     -sin(h) 0 cos(h)];
end
 
function R = RZ(h)
R = [cos(h) -sin(h) 0;
     sin(h) cos(h) 0;
     0 0 1];
end

function R = R_ZYX(theta)
R = RZ(theta(1))*RY(theta(2))*RX(theta(3));
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

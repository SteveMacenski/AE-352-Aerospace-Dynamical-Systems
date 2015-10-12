function hw2
%
%
% To complete Problems 3-4, you need only change the lines in this script
% that are marked "<-- CHANGE THIS LINE (PROBLEM XXXX)".
%
%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% INITIALIZATION
%

%%%%
% STUFF YOU SHOULD NOT CHANGE
%
% Define the geometry of everything.
[pRobot_in3,fRobot,pFrame0_in0,pFrame3_in3,pLight_in0,pPoint_in3] = GetGeometry;
% Create figure.
world = CreateFigure(pRobot_in3,fRobot,pFrame0_in0,pFrame3_in3,pLight_in0,pPoint_in3);
% Define time at which to start the simulation.
t = 0;
% Define time step.
dt = 1e-2;
%
%%%%

%%%%
% STUFF YOU MUST CHANGE
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
o_3in0 = [0;0;0];    % <-- CHANGE THIS LINE (PROBLEM 4(a,b,c).i)
theta = [0;0;0];     % <-- CHANGE THIS LINE (PROBLEM 4(a,b,c).i)
%
% Define time at which to stop the simulation.
%
%   tmax    a number, the time at which the simulation will end
%
tmax = 2*pi;               % <-- CHANGE THIS LINE (PROBLEM 4(a,b,c).ii)
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
    [t,o_3in0,theta] = Simulate(t,o_3in0,theta,dt);
    % Get linear and angular velocity.
    odot_3in0 = GetLinearVelocity(t);
    w03_in3 = GetAngularVelocity(t);
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
    
    R_3in0 = [cos(theta2)*cos(theta3) -cos(theta2)*sin(theta3) sin(theta2);
              cos(theta1)*sin(theta3)+cos(theta3)*sin(theta1)*sin(theta2) cos(theta1)*cos(theta3)-sin(theta1)*sin(theta2)*sin(theta3) -cos(theta2)*sin(theta1);
              sin(theta1)*sin(theta3)-cos(theta1)*cos(theta3)*sin(theta2) cos(theta3)*sin(theta1)+cos(theta1)*cos(theta1)*sin(theta2)*sin(theta3) cos(theta1)*cos(theta2)];    % <-- CHANGE THIS LINE (PROBLEM 3(a))
    %
    %   R_0in3  the orientation of frame 0 in the coordinates of frame 3
    %           (this comes from an inverse transformation)
    %
    R_0in3 = transpose(R_3in0);    % <-- CHANGE THIS LINE ((b))
    %
    %   o_0in3  the position of frame 0 in the coordinates of frame 3
    %           (this comes from an inverse transformation)
    %
    o_0in3 = -R_0in3*o_3in0;   % <-- CHANGE THIS LINE (PROBLEM 3(b))
    %
    % (2) Compute position and velocity of the point on the spacecraft in
    %     frames 0 and 3
    %
    % Given:
    %
    %   o_3in0      the position of frame 3 in the coordinates of frame 0
    %
    %   R_3in0      the orientation of frame 3 in the coordinates of frame 0
    %
    %   odot_3in0   the linear velocity of frame 3 in the coordinates of
    %               frame 0
    %
    %   w03_in3     the angular velocity of frame 3 with respect to frame
    %               0, written in the coordinates of frame 3
    %
    % Compute:
    %
    %   pPoint_in3      position of point on spacecraft in frame 3
    %
    pPoint_in3 = [.3*cos(2*t);.3*sin(2*t);.125];       % <-- CHANGE THIS LINE (PROBLEM 4(a,b,c).iii)
    %
    %   pPointDot_in3   velocity of point on spacecraft in frame 3
    %
    pPointDot_in3 = [-.6*sin(2*t);.6*sin(2*t);0];    % <-- CHANGE THIS LINE (PROBLEM 4(a,b,c).iii)
    %
    %   pPointDot_in0   velocity of point on spacecraft in frame 0
    %
    pPointDot_in0 = odot_3in0 + R_3in0*[0 -w03_in3(3) w03_in3(2); w03_in3(3) 0 -w03_in3(1); -w03_in3(2) w03_in3(1) 0]*pPoint_in3 + transpose(R_3in0)*pPointDot_in3;    % <-- CHANGE THIS LINE (PROBLEM 3(e))
    %
    % (3) Transform stuff from frame 3 to frame 0
    %
    % Given:
    %
    %   o_3in0  the position of frame 3 in the coordinates of frame 0
    %
    %   R_3in0  the orientation of frame 3 in the coordinates of frame 0
    %
    %   pRobot_in3, pFrame3_in3, pPoint_in3
    %       - all matrices of size 3 x <something>
    %       - each column has the coordinates of a point in frame 3
    %
    % Compute:
    %
    %   pRobot_in0, pFrame3_in0, pPoint_in0
    %       - all matrices of size 3 x <something>
    %       - each column has the coordinates of a point in frame 0
    %
    pRobot_in0 = R_3in0*pRobot_in3 + [o_3in0(1) o_3in0(1) o_3in0(1) o_3in0(1) o_3in0(1) o_3in0(1) o_3in0(1) o_3in0(1); o_3in0(2) o_3in0(2) o_3in0(2) o_3in0(2) o_3in0(2) o_3in0(2) o_3in0(2) o_3in0(2);o_3in0(3) o_3in0(3) o_3in0(3) o_3in0(3) o_3in0(3) o_3in0(3) o_3in0(3) o_3in0(3)];        % <-- CHANGE THIS LINE (PROBLEM 3(c))
    pFrame3_in0 = R_3in0*pFrame3_in3 + [ o_3in0(1) o_3in0(1) o_3in0(1) o_3in0(1); o_3in0(2) o_3in0(2) o_3in0(2) o_3in0(2); o_3in0(3) o_3in0(3) o_3in0(3) o_3in0(3)];      % <-- CHANGE THIS LINE (PROBLEM 3(c))
    pPoint_in0 = R_3in0*pPoint_in3 + o_3in0;      % <-- CHANGE THIS LINE (PROBLEM 3(c))
    %
    % (4) Transform stuff from frame 0 to frame 3
    %
    % Given:
    %
    %   o_0in3  the position of frame 0 in the coordinates of frame 3
    %
    %   R_0in3  the orientation of frame 0 in the coordinates of frame 3
    %
    %   pFrame0_in0, pLight_in0
    %       - all matrices of size 3 x <something>
    %       - each column has the coordinates of a point in frame 0
    %
    % Compute:
    %
    %   pFrame0_in3, pLight_in3
    %       - all matrices of size 3 x <something>
    %       - each column has the coordinates of a point in frame 3
    %
    pFrame0_in3 = R_0in3* pFrame0_in0;      % <-- CHANGE THIS LINE (PROBLEM 3(c))
    pLight_in3 = R_0in3*pLight_in0;        % <-- CHANGE THIS LINE (PROBLEM 3(c))
    %
    %%%%
    
    %%%%
    % STUFF YOU SHOULD NOT CHANGE
    %
    % Update the figure.
    world = UpdateFigure(world,pRobot_in0,fRobot,pFrame0_in0,...
                            pFrame3_in0,pPoint_in0,pPointDot_in0,...
                            pRobot_in3,pFrame0_in3,pFrame3_in3,...
                            pPoint_in3,pPointDot_in3,pLight_in3,o_3in0,...
                            R_3in0,theta,w03_in3,odot_3in0,t,tmax);
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

function thetadot = GetThetaDot(theta,w03_in3)
%
%   theta       is a 3x1 matrix with the XYZ Euler Angles (in radians)
%
%   w03_in3     is the angular velocity of frame 3 with respect to frame 0,
%               written in the coordinates of frame 3
%
%   thetadot    is a 3x1 matrix with the XYZ Euler Angular Rates (in
%               radians/second)
theta2 = theta(2);
theta1 = theta(1);
theta3 = theta(3);
%
thetadot = inv([cos(theta2)*cos(theta3), sin(theta3), 0; -cos(theta2)*sin(theta3), cos(theta3), 0; sin(theta2), 0, 1])*w03_in3;     % <-- CHANGE THIS LINE (PROBLEM 3(d))

function odot_3in0 = GetLinearVelocity(t)
%
%   t           is the current time
%
%   odot_3in0   is the time derivative of the position of frame 3 in the
%               coordinates of frame 0
%
odot_3in0 = [0;0;0];    % <-- CHANGE THIS LINE (PROBLEM 4(a,b,c).iv)

function w03_in3 = GetAngularVelocity(t)
%
%   t           is the current time
%
%   w03_in3     is the angular velocity of frame 3 with respect to frame 0,
%               written in the coordinates of frame 3
%
w03_in3 = 10*exp(-t)*[sin(t);sin(2*t);sin(3*t)];    % <-- CHANGE THIS LINE (PROBLEM 4(a,b,c).iv)

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS YOU SHOULD NOT CHANGE
%

function [t,o_3in0,theta] = Simulate(t,o_3in0,theta,dt)
[t,q] = ode45(@GetQDot,[t t+dt],[o_3in0;theta]);
q = q';
t = t(end);
o_3in0 = q(1:3,end);
theta = q(4:6,end);

function qdot = GetQDot(t,q)
[odot_3in0,thetadot] = GetRates(t,q(1:3,:),q(4:6,:));
qdot = [odot_3in0;thetadot];

function [odot_3in0,thetadot] = GetRates(t,o_3in0,theta)
odot_3in0 = GetLinearVelocity(t);
w03_in3 = GetAngularVelocity(t);
thetadot = GetThetaDot(theta,w03_in3);

function [pRobot_in3,fRobot,pFrame0_in0,pFrame3_in3,pLight_in0,...
            pPoint_in3,pPointTrace_in0,pPointTrace_in3] = GetGeometry()
% - The spacecraft (a box).
L=1.25;
W=0.75;
H=0.25;
pRobot_in3 = [-L/2  L/2  L/2 -L/2 -L/2  L/2  L/2 -L/2;
              -W/2 -W/2 -W/2 -W/2  W/2  W/2  W/2  W/2;
              -H/2 -H/2  H/2  H/2 -H/2 -H/2  H/2  H/2];
fRobot = [1 2 3 4;
          2 6 7 3;
          6 5 8 7;
          5 1 4 8;
          4 3 7 8;
          5 6 2 1];
% - The reference frames.
pFrame0_in0 = [0 1 0 0;
               0 0 1 0;
               0 0 0 1];
pFrame3_in3 = [0 1 0 0;
               0 0 1 0;
               0 0 0 1];
% - The lights.
pLight_in0 = [ 0 -2;
              -4  2;
               2  4];
% - The point on the surface of the spacecraft (just a placeholder).
pPoint_in3 = [nan;nan;nan];
pPointTrace_in0 = [];
pPointTrace_in3 = [];

function vector = DrawVector(vector,p,v)
if (isempty(vector))
    vector = plot3(p(1)+[0 v(1)],p(2)+[0 v(2)],p(3)+[0 v(3)],'m-');
else
    set(vector,'xdata',p(1)+[0 v(1)],'ydata',p(2)+[0 v(2)],'zdata',p(3)+[0 v(3)]);
end

function point = DrawPoint(point,p)
if (isempty(point))
    point.dot = plot3(p(1,end),p(2,end),p(3,end),'k.','markersize',16);
    point.trace = plot3(p(1,:),p(2,:),p(3,:),'k-');
else
    set(point.dot,'xdata',p(1,end),'ydata',p(2,end),'zdata',p(3,end));
    set(point.trace,'xdata',p(1,:),'ydata',p(2,:),'zdata',p(3,:));
end

function robot = DrawRobot(robot,p,f)
if isempty(robot)
    robot = patch('Vertices',p','Faces',f,...
                  'FaceColor','y','FaceAlpha',0.6,'EdgeAlpha',0.6);
else
    set(robot,'vertices',p');
end

function frame = DrawFrame(frame,p)
if isempty(frame)
    frame.x = plot3(p(1,[1 2]),p(2,[1 2]),p(3,[1 2]),'r-','linewidth',3);
    frame.y = plot3(p(1,[1 3]),p(2,[1 3]),p(3,[1 3]),'g-','linewidth',3);
    frame.z = plot3(p(1,[1 4]),p(2,[1 4]),p(3,[1 4]),'b-','linewidth',3);
else
    set(frame.x,'xdata',p(1,[1 2]),'ydata',p(2,[1 2]),'zdata',p(3,[1 2]));
    set(frame.y,'xdata',p(1,[1 3]),'ydata',p(2,[1 3]),'zdata',p(3,[1 3]));
    set(frame.z,'xdata',p(1,[1 4]),'ydata',p(2,[1 4]),'zdata',p(3,[1 4]));
end

function world = CreateFigure(pRobot_in3,fRobot,pFrame0_in0,pFrame3_in3,pLight_in0,pPoint_in3)
% - clear the current figure
clf;
% - text
axes('position',[0 0 1 1]);
axis([0 1 0 1]);
hold on;
axis off;
fs = 10;
world.text.olabel=text(0.075,0.9,'o_3^0 =','fontsize',fs,'fontname','monaco');
world.text.o=text(0.125,0.92,sprintf('%6.2f\n',zeros(3,1)),'fontsize',fs,'verticalalignment','top','fontname','monaco');
world.text.Rlabel=text(0.25,0.9,'R_3^0 =','fontsize',fs,'fontname','monaco');
world.text.R=text(0.3,0.92,sprintf('%6.2f %6.2f %6.2f\n',eye(3)),'fontsize',fs,'verticalalignment','top','fontname','monaco');
world.text.roll=text(0.6,0.94,sprintf(' roll:   %s = %4.0f','\theta_1',0),'fontsize',fs,'verticalalignment','top','fontname','monaco','visible','on');
world.text.pitch=text(0.6,0.89,sprintf('pitch:   %s = %4.0f','\theta_2',0),'fontsize',fs,'verticalalignment','top','fontname','monaco','visible','on');
world.text.yaw=text(0.6,0.84,sprintf('  yaw:   %s = %4.0f','\theta_3',0),'fontsize',fs,'verticalalignment','top','fontname','monaco','visible','on');
world.text.odotlabel=text(0.05,0.13,'$\dot{o}_3^0 =$','interpreter','latex','fontsize',14,'fontname','monaco');
world.text.odot=text(0.125,0.15,sprintf('%6.2f\n',zeros(3,1)),'fontsize',fs,'verticalalignment','top','fontname','monaco');
world.text.wlabel=text(0.3,0.13,'$w_{0,3}^3 =$','interpreter','latex','fontsize',14,'fontname','monaco');
world.text.w=text(0.390,0.15,sprintf('%6.2f\n',zeros(3,1)),'fontsize',fs,'verticalalignment','top','fontname','monaco');
world.text.rolldotlabel=text(0.6,0.185,'$\dot{\theta}_1 =$','interpreter','latex','fontsize',14,'fontname','monaco');
world.text.pitchdotlabel=text(0.6,0.12,'$\dot{\theta}_2 =$','interpreter','latex','fontsize',14,'fontname','monaco');
world.text.yawdotlabel=text(0.6,0.06,'$\dot{\theta}_3 =$','interpreter','latex','fontsize',14,'fontname','monaco');
world.text.thetadot=text(0.65,0.2,sprintf('%6.2f\n\n',zeros(3,1)),'fontsize',fs,'verticalalignment','top','fontname','monaco');
world.text.time=text(0.35,0.75,sprintf('t = %6.2f / %6.2f\n',0,0),'fontsize',fs,'verticalalignment','top','fontname','monaco');
% - view from frame 0
axes('position',[0.05 0.25 0.35 0.5]);
title('view: frame 0');
set(gcf,'renderer','opengl');
axis equal;
axis(1.5*[-1 1 -1 1 -1 1]);
axis manual;
hold on;
view([90-37.5,20]);
box on;
set(gca,'projection','perspective');
lighting gouraud
world.view0.light(1) = light('Position',pLight_in0(:,1)');
world.view0.light(2) = light('Position',pLight_in0(:,2)');
world.view0.robot = DrawRobot([],pRobot_in3,fRobot);
world.view0.frame0 = DrawFrame([],pFrame0_in0);
world.view0.frame3 = DrawFrame([],pFrame3_in3);
world.view0.point = DrawPoint([],pPoint_in3);
world.view0.pointdot = DrawVector([],[0;0;0],[0;0;0]);
world.pPointTrace_in0=[];
% - view from frame 3
axes('position',[0.6 0.25 0.3 0.5]);
title('view: frame 3');
set(gcf,'renderer','opengl');
axis equal;
axis(1.5*[-1 1 -1 1 -1 1]);
axis manual;
hold on;
box on;
lighting gouraud
world.view3.light(1) = light('Position',pLight_in0(:,1)');
world.view3.light(2) = light('Position',pLight_in0(:,2)');
campos([0;0;5]);
camtarget([0;0;-5]);
camup([-1;0;0]);
camroll(90);
world.view3.robot = DrawRobot([],pRobot_in3,fRobot);
world.view3.frame0 = DrawFrame([],pFrame0_in0);
world.view3.frame3 = DrawFrame([],pFrame3_in3);
world.view3.point = DrawPoint([],pPoint_in3);
world.view3.pointdot = DrawVector([],[0;0;0],[0;0;0]);
world.pPointTrace_in3=[];

function world = UpdateFigure(world,pRobot_in0,fRobot,pFrame0_in0,...
    pFrame3_in0,pPoint_in0,pPointDot_in0,pRobot_in3,...
    pFrame0_in3,pFrame3_in3,pPoint_in3,pPointDot_in3,...
    pLight_in3,o_3in0,R_3in0,theta,w03_in3,odot_3in0,t,tmax)
world.pPointTrace_in0(:,end+1) = pPoint_in0;
world.pPointTrace_in3(:,end+1) = pPoint_in3;
world.view0.robot = DrawRobot(world.view0.robot,pRobot_in0,fRobot);
world.view0.frame0 = DrawFrame(world.view0.frame0,pFrame0_in0);
world.view0.frame3 = DrawFrame(world.view0.frame3,pFrame3_in0);
world.view0.point = DrawPoint(world.view0.point,world.pPointTrace_in0);
world.view0.pointdot = DrawVector(world.view0.pointdot,pPoint_in0,pPointDot_in0);
world.view3.robot = DrawRobot(world.view3.robot,pRobot_in3,fRobot);
world.view3.frame0 = DrawFrame(world.view3.frame0,pFrame0_in3);
world.view3.frame3 = DrawFrame(world.view3.frame3,pFrame3_in3);
world.view3.point = DrawPoint(world.view3.point,world.pPointTrace_in3);
world.view3.pointdot = DrawVector(world.view3.pointdot,pPoint_in3,pPointDot_in3);
for i=1:2
    set(world.view3.light(i),'position',pLight_in3(:,i)');
end
set(world.text.o,'string',sprintf('%6.2f\n',o_3in0));
set(world.text.R,'string',sprintf('%6.2f %6.2f %6.2f\n',R_3in0'));
set(world.text.roll,'string',sprintf(' roll:   %s = %4.0f','\theta_1',theta(1)*180/pi));
set(world.text.pitch,'string',sprintf('pitch:   %s = %4.0f','\theta_2',theta(2)*180/pi));
set(world.text.yaw,'string',sprintf('  yaw:   %s = %4.0f','\theta_3',theta(3)*180/pi));
set(world.text.w,'string',sprintf('%6.2f\n',w03_in3));
set(world.text.thetadot,'string',sprintf('%6.2f\n\n',GetThetaDot(theta,w03_in3)));
set(world.text.odot,'string',sprintf('%6.2f\n',odot_3in0));
set(world.text.time,'string',sprintf('t = %6.2f / %6.2f\n',t,tmax));
drawnow

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

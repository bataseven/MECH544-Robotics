%% Run the simulation and get the outputs
clear
close all
clc

P_start = [0; 0; 0];
P_intermediate = [0.1; 0.01; 0.04];
P_goal = [0.15; 0.02; 0.05];

% P_start = [0; 0; 0];
% P_intermediate = [0.1; 0; 0.04];
% P_goal = [0.15; 0; 0.05];

simulation_time = 6;
out = sim('Project03_Final',simulation_time);
t = out.tout;

%% Coordinate positions with time
figure
subplot(3,1,1);
hold on
grid on
plot(t,out.des_x,'-.b');
plot(t,out.out_x,'r');
legend('Desired x-coordinate', 'Actual x-coordinate','location','northwest');
ylabel('x');
xlabel('time (s)');

subplot(3,1,2);
hold on
grid on
plot(t,out.des_y,'-.b');
plot(t,out.out_y,'r');
legend('Desired y-coordinate', 'Actual y-coordinate','location','northwest');
ylabel('y');
xlabel('time (s)');

subplot(3,1,3);
hold on
grid on
plot(t,out.des_z,'-.b');
plot(t,out.out_z,'r');
legend('Desired z-coordinate', 'Actual z-coordinate','location','northwest');
ylabel('z');
xlabel('time (s)');
sgtitle('Coordinates With Time');
%% Force outputs
border_x = [0 simulation_time];
border_y = [-1 -1];

figure
hold on
plt = plot(t,out.Fx);
line(border_x,border_y,'Color','red','LineStyle','--');
border_line = line(border_x,-border_y,'Color','red','LineStyle','--');
legend([border_line plt],'1N Limit','Applied Force');
title({'End-Effector Applied Force','(X Component)'});
ylabel('Force (N)');
xlabel('Time (s)');

figure
hold on
plt = plot(t,out.Fy);
line(border_x,border_y,'Color','red','LineStyle','--');
border_line = line(border_x,-border_y,'Color','red','LineStyle','--');
legend([border_line plt],'1N Limit','Applied Force');
title({'End-Effector Applied Force','(Y Component)'});
ylabel('Force (N)');
xlabel('Time (s)');

figure
hold on
plt = plot(t,out.Fz);
line(border_x,border_y,'Color','red','LineStyle','--');
border_line = line(border_x,-border_y,'Color','red','LineStyle','--');
legend([border_line plt],'1N Limit','Applied Force');
title({'End-Effector Applied Force','(Z Component)'});
ylabel('Force (N)');
xlabel('Time (s)');
%% Trajectory
figure
grid on;
hold on;

plot3(out.des_x,out.des_y,out.des_z,'r','MarkerSize',1);
plot3(out.out_x,out.out_y,out.out_z,'k','MarkerSize',1);
legend('Desired Path','Actual Path');
xlabel('Z');
ylabel('X');
zlabel('Y');
%% Animate
theta1 = out.theta1;
theta2 = out.theta2;
theta3 = out.theta3;

L1 = 0.215;
L2 = 0.170;

figH = figure('Position', [50 100 1200 500]);
subplot(1,2,1);
axis equal
axis manual
grid on
hold on
trj = plot([P_start(3) P_intermediate(3) P_goal(3)],[P_start(2) P_intermediate(2) P_goal(2)],'r');
xlim([-0.3 0.3]);
ylim([-0.1 0.5]);
xlabel('Z')
ylabel('Y');
title('Side View (Y-Z Plane)');
legend(trj,'Trajectory','AutoUpdate','off')

subplot(1,2,2);
axis equal
axis manual
grid on
hold on
trj = plot([P_start(3) P_intermediate(3) P_goal(3)],[P_start(1) P_intermediate(1) P_goal(1)],'r');
xlim([-0.3 0.3]);
ylim([-0.1 0.5]);
xlabel('Z')
ylabel('X');
title('Top View (X-Z Plane)');
legend(trj,'Trajectory','AutoUpdate','off')
for i = 1 : 40 : length(t)
    
    % Side view links
    
    x1_side = cos(theta1(i)) * (cos(theta2(i)) * L1) - L1;
    x1_side = linspace(-L1,x1_side, 2);
    
    y1_side = cos(theta1(i)) * (sin(theta2(i)) * L1) + L2;
    y1_side = linspace(L2,y1_side,2);
    
    x2_side = out.out_z(i);
    x2_side = linspace(x1_side(end),x2_side,2);
    
    y2_side = out.out_y(i);
    y2_side = linspace(y1_side(end),y2_side,2);
    
    % Top view links
    link1_tip_x = cos(theta2(i)) * cos(theta1(i)) * L1 - L1;
    link1_tip_x = [-L1 link1_tip_x];
    
    link1_tip_y = cos(theta2(i)) * sin(theta1(i)) * L1;
    link1_tip_y = [0 link1_tip_y];
    
    x2_top = out.out_z(i);
    x2_top = linspace(link1_tip_x(end),x2_top,2);
    
    y2_top = out.out_x(i);
    y2_top = linspace(link1_tip_y(end),y2_top,2);
    
    if ~ishghandle(figH)
        break
    end
    
    subplot(1,2,1);
    link1_side = line(x1_side,y1_side,'Color','green','LineWidth',2);
    link2_side = line(x2_side,y2_side,'Color','blue','LineWidth',2);
    txt = text(5,95,strcat("Time: ", num2str(t(i))));
    
    subplot(1,2,2);
    link1_top = line(link1_tip_x,link1_tip_y,'Color','green','LineWidth',2);
    link2_top = line(x2_top,y2_top,'Color','blue','LineWidth',2);
    drawnow
    
    delete(link1_side);
    delete(link2_side);
    delete(link1_top);
    delete(link2_top);
    delete(txt);
end
subplot(1,2,1);
link1_side = line(x1_side,y1_side,'Color','green','LineWidth',2);
link2_side = line(x2_side,y2_side,'Color','blue','LineWidth',2);
txt = text(5,95,strcat("Time: ", num2str(t(i))));

subplot(1,2,2);
link1_top = line(link1_tip_x,link1_tip_y,'Color','green','LineWidth',2);
link2_top = line(x2_top,y2_top,'Color','blue','LineWidth',2);
drawnow
%% Errors
figure
plot(t,out.error_x);
title('X Error');

figure
plot(t,out.error_y);
title('Y Error');

figure
plot(t,out.error_z);
title('Z Error');

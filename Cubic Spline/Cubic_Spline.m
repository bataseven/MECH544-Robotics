% Berke Ataseven
% ID: 54326
% MECH544 HW Assignment CH07
% This program generates cubic splines between given data points and
% compares them with the built-in function of MATLAB

close all
clear
clc



time  = [0  2  3  6  8]; % Put time values in an array
angle = [10 35 25 30 60]; % Put angle values in an array

% angle = sort((80)*rand(100,1),'ascend')';
% time = linspace(0,100, 100);


grid on
hold on

plot(time, angle,'*k','MarkerSize',5,'LineWidth',1.5); % Plot the via points
plot(time, angle,'-g'); % Plot the linear interpolation

cubic_spline = generate_cubic_spline(time, angle); % Generate cubic spline
plot(cubic_spline(:,1),cubic_spline(:,2),'-r','LineWidth',1.5); % Plot the custom cubic spline

xx = 0 : 0.01 : max(time);
yy = spline(time, angle,xx); % Built-in MATLAB spline
plot(xx,yy,'b','LineWidth',1)

title('Joint Angle vs Time');
xlabel('Time (s)');
ylabel(['Angle (\theta' char(176) ')']);
ylim([0 70]);
legend('Via Points','Linear Interpolation','Custom Spline','Built-in MATLAB Spline','Location','best');


function spline = generate_cubic_spline(x, f_x) % Generates segments of continuous cubic splines between data points given the values of x and f(x)
times = x;
thetas = f_x;

dt = 0.01; % time increment
spline = []; % This will be the value to return

N = length(thetas); % Number of nodes connected by segments
SN = N - 1; % Number of segments

Average_Segment_Velocities = zeros(1,SN); % Define average velocities for each segment

for i = 1 : SN
    Average_Segment_Velocities(i) = (thetas(i+1) - thetas(i)) / (times(i+1) - times(i));  % Average velocity of a segment
end

initial_theta_dot = 0; % Initial and final node velocities are given as 0
final_theta_dot = 0; % Initial and final node velocities are given as 0

theta_dots = ones(N,1); % Create the array that will store the node velocities
theta_dots(1) = initial_theta_dot; % Initial and final node velocities are given as 0
theta_dots(N) = final_theta_dot; % Initial and final node velocities are given as 0

for i = 2 : N - 1
    % If the slope is changing from "-" to "+" or "+" to "-", then set the node velocity to zero.
    if Average_Segment_Velocities(i-1) * Average_Segment_Velocities(i) <= 0
        theta_dots(i) = 0;
        
        %Else take the average velocity of 2 consecutive splines,
        %then set node velocity to that value.
    else
        theta_dots(i) = (Average_Segment_Velocities(i-1) + Average_Segment_Velocities(i)) / 2;
    end
end

theta_0 = thetas(1:end-1); % Array of theta zeros
theta_f = thetas(2:end); % Array of theta finals
theta_dot_0 = theta_dots(1:end-1); % Array of theta_dot zeros
theta_dot_f = theta_dots(2:end); % Array of theta_dot finals

for i = 1 : SN % Iterate for number of segments
    tf = times(i+1) - times(i); % Normalized final time
    
    a0 = theta_0(i);
    a1 = theta_dot_0(i);
    a2 = 3 * (theta_f(i) - theta_0(i)) / tf ^ 2 - 2 * theta_dot_0(i) / tf - theta_dot_f(i) / tf;
    a3 = -2 * (theta_f(i) - theta_0(i)) / tf ^ 3 +  (theta_dot_f(i) + theta_dot_0(i)) / tf ^ 2 ;
    
    t = (0 : dt : tf - dt)';  % Create discretized time points
    cubic_poly = a3 * t.^3 + a2 * t.^2 + a1 * t + a0; % Spline segment
    
    spline = [spline; [t + times(i) cubic_poly]]; % Append the spline segment (de-normalize by adding times(i) to t)
end
spline = [spline; [times(end) thetas(end)]]; % Append the last element, since it is not included in the for loop (line 82: tf - dt)
end
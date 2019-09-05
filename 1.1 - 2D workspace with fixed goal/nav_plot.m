%% nav_plot

%% DESCRIPTION

%{
name: nav_plot

type: function

input: 'W_x', 'W_y' workspace coordinates; 'x_obs', 'r_obs' obstacle
positions, radii; 'x_c_cat', 'x_hat_cat' robot position and estimated
projected goal log over time

output: a plot of the robot navigating through the workspace including
circular obstacles is produced - the green line is the robot's estimated
projected goal from 'x_d' while the blue line is the path taken by the
robot

purpose: see 'output'

author:     Leopold Thebault, leopold@thebault.lt; lthebaul@caltech.edu;
s1613153@sms.ed.ac.uk

date: 2019.05.30

updated: 2019.05.30
%}

%% NOTES

%{
%}

%% CODE

function[] = nav_plot(W_x,W_y,x_obs,r_obs,x_c_cat,x_hat_cat)

% Number of obstacles
m               = size(x_obs,2);

% Plot workspace
plot(W_x, W_y, 'k', 'Linewidth', 2);
axis equal;
axis ([-20 20 -20 20]);
hold on;

% Plot obstacles
for i = 1:m
    theta       = linspace(0,2*pi);
    x           = r_obs(i)*cos(theta) + x_obs(1,i);
    y           = r_obs(i)*sin(theta) + x_obs(2,i);
    plot(x,y,'k');
    hold on;
end

% Plot projected goal and path
plot(x_c_cat(1,:), x_c_cat(2,:), 'b');
plot(x_c_cat(1,1), x_c_cat(2,1), 'k-o');
plot(x_c_cat(1,end), x_c_cat(2,end), 'k-o');
text(x_c_cat(1,1)-1.5, x_c_cat(2,1)-1.5, 'x_c', 'FontSize', 18);
text(x_c_cat(1,end)+1, x_c_cat(2,end)+1, 'x_d', 'FontSize', 18);
plot(x_hat_cat(1,:), x_hat_cat(2,:), 'g');
title('Robot navigation with fixed target', 'FontSize', 24);

hold off;
xlabel('X', 'FontSize', 24)
ylabel('Y', 'FontSize', 24)

end

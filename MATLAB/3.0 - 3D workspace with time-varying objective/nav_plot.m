%% plot

%% DESCRIPTION

%{
name: nav_plot

type: function

input: 'x_obs', 'r_obs' obstacle
positions, radii; 'x_c_cat', 'x_hat_cat' robot position and estimated
projected goal log over time

output: a plot of the robot navigating through the workspace including
circular obstacles is produced - the green line is the robot's estimated
projected goal from 'x_d', the blue line is the path taken by the
robot, the red line is the path followed by the target 'x_d' over time

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

function[] = nav_plot(W,x_obs,r_obs,x_c_cat,x_hat_cat,x_d_cat)

% Number of obstacles
m               = size(x_obs,2);

for i = 1:m
  % Generate a sphere consisting of 20x20 faces
  [x,y,z]       = sphere;
  % Use 'surf' function to plot
  hSurface      = surf(r_obs(i)*x+x_obs(1,i),r_obs(i)*y+x_obs(2,i), ...
      r_obs(i)*z+x_obs(3,i));
  hold on
  set(hSurface,'FaceColor',[0 0 0], ...
      'FaceAlpha',0.5,'FaceLighting','gouraud','EdgeColor','none')
  axis(W.*[-1 1 -1 1 -1.25 .5]);
  daspect([1 1 1]);
end

plot3(x_c_cat(1,:), x_c_cat(2,:), x_c_cat(3,:), 'b');
plot3(x_hat_cat(1,:), x_hat_cat(2,:), x_hat_cat(3,:), 'g');
plot3(x_d_cat(1,:), x_d_cat(2,:), x_d_cat(3,:), 'r');
plot3(x_c_cat(1,1), x_c_cat(2,1), x_c_cat(3,1), 'k-o');
text(x_c_cat(1,1)-1, x_c_cat(2,1)-1, x_c_cat(3,1)-1, 'x_c', ...
    'FontSize', 18);
plot3(x_d_cat(1,1), x_d_cat(2,1), x_d_cat(3,1), 'k-o');
text(x_d_cat(1,1)+1, x_d_cat(2,1)+1, x_d_cat(3,1)+1, 'x_d', ...
    'FontSize', 18);
title('3D robot navigation with moving target', 'FontSize', 24);

hold off
xlabel('X', 'FontSize', 24)
ylabel('Y', 'FontSize', 24)
zlabel('Z', 'FontSize', 24)
camlight

end

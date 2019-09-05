%% workspace

%% DESCRIPTION

%{
name: workspace

type: function

input: none - workspace dimensions and positions are defined in this
function (hard-coded)

output: 'W_x', 'W_y' workspace coordinates; 'x_obs', 'r_obs' obstacle
positions, radii

purpose: this function simply defines the workspace and obstacles around
which the robot will have to navigate

author:     Leopold Thebault, leopold@thebault.lt; lthebaul@caltech.edu;
s1613153@sms.ed.ac.uk

date: 2019.05.30

updated: 2019.05.30
%}

%% NOTES

%{
- could use 'viscircles' or 'rectangle' with rounded edges and 'Facecolor'
to create and plot cirlces
%}

%% CODE

function[W_x,W_y,x_obs,r_obs] = workspace()

% Workspace boundaries
W_x             = [-20 20 20 -20 -20];
W_y             = [-20 -20 20 20 -20];

% Obstacle defintion
x_obs           = [-15 -10 -15 -5 5 5 15 10 0 0 -5 10; ...
    -15 -5 10 5 -15 5 10 -5 -5 15 -15 15];
r_obs           = [2 4 3 2 3 3 2 3 2 2 3 2];

end

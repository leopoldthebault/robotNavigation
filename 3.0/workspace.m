%% workspace

%% DESCRIPTION

%{
name: workspace

type: function

input: none - workspace dimensions and positions are defined in this
function (hard-coded)

output: 'x_obs', 'r_obs' obstacle
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

function[W,x_obs,r_obs] = workspace()

% Edge size of workspace
W               = 20;

% Obstacle defintion
x_obs           = [10 10 5 -10 5 -10; ...
    -5 10 -10 5 15 -10; ...
    0 -7.5 -10 -5 0 -15];
r_obs           = [3 3 4 3 3 3];

end
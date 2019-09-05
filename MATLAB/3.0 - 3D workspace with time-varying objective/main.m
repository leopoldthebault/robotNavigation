%% main

%% DESCRIPTION

%{
name: main

type: function

input: 'r', 'x_c' robot radius, initial position, destination/goal; 'r_d'
radius of target/goal displacement; 't_f', 'delta_t' number of iterations
to compute in ODE solution, time-step for ODE solution; 'K' controller
gain; 'alpha' correction term coefficient

output: a plot of the robot navigating through the workspace including
circular obstacles is produced - the green line is the robot's estimated
projected goal from 'x_d', the blue line is the path taken by the
robot, the red line is the path followed by the target 'x_d' over time;
total computation time is also given

purpose: this program is a direct application of the time-dependent target
robot navigation program from "Prediction-Correction Interior-Point Method
for Time-Varying Convex Optimization" [IV,D,1:2] by Mayhar Fazylab et al. -
definitions for all variables in this code can be found within said paper

author:     Leopold Thebault, leopold@thebault.lt; lthebaul@caltech.edu;
s1613153@sms.ed.ac.uk

date: 2019.05.30

updated: 2019.05.30
%}

%% NOTES

%{
- all arguments to 'main' have been hard-coded to make the function trivial
to run
%}

%% CODE

function[] = main() % main(r,x_c,r_d,t_f,delta_t,K,alpha)

clear; clc;

% profile on;

% Arguments to 'main' - hard-coded for simplicity
r = 1; x_c = [-15;-15;-15];
t_f = 1000; delta_t = 0.01;
K = 0.1; alpha = 10;

% CPU time registered to determine total computation time
cpu_t           = cputime;

% Call 'workspace' function which defines the workspace and obstacles
[W,x_obs,r_obs]   = workspace();

% Creation of matrices used to log the robot position, estimated projected
% goal over time, and time varying target/goal position
x_hat           = x_c;
x_c_cat         = [];
x_hat_cat       = [];
x_d_cat         = [];

% Solution of ODEs through basic implementation of Euler's method with
% constant time-steps
for t = 1:delta_t:t_f
    % Target defined as time-varying based on CPU time elapsed
    dx          = 0.25*(cputime - cpu_t);
    x_d         = -.75*W.*[sin(dx).*cos(10*dx); sin(dx).*sin(10*dx); ...
        cos(dx)];
    x_c_cat     = cat(2,x_c_cat,x_c);
    x_hat_cat   = cat(2,x_hat_cat,x_hat);
    x_d_cat     = cat(2,x_d_cat,x_d);
    % Update constraint parameters
    [a,b,a_dot,b_dot] = constraint(x_obs,r_obs,x_c,r,x_hat,K);
    % Update estimated projected goal
    x_hat       = x_hat + delta_t.*ode(x_hat,x_d,t,a,b,a_dot,b_dot, ...
        alpha,x_obs);
    % Act on robot position based on controller parameters and updated
    % estimated projected goal
    x_c         = x_c + delta_t.*(-K.*(x_c - x_hat));
end

% Plot through dedicated function
nav_plot(W,x_obs,r_obs,x_c_cat,x_hat_cat,x_d_cat);

% Display elapsed time during computation
disp(['Elasped time: ',num2str(cputime - cpu_t)]);

% profile off;
% profile viewer;

end

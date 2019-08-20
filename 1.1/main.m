%% main

%% DESCRIPTION

%{
name: main

type: function

input: 'r', 'x_c', 'x_d' robot radius, initial position, destination/goal;
'T', 'delta_t' number of iterations to compute in ODE solution, time-step
for ODE solution; 'K' controller gain; 'alpha' correction term coefficient

output: a plot of the robot navigating through the workspace including
circular obstacles is produced - the green line is the robot's estimated
projected goal from 'x_d' while the blue line is the path taken by the
robot; total computation time

purpose: this program is a direct application of the fixed-target robot
navigation program from "Prediction-Correction Interior-Point Method for
Time-Varying Convex Optimization" [IV,D,1] by Mayhar Fazylab et al. -
definitions for all variables in this code can be found within said paper

author:     Leopold Thebault, leopold@thebault.lt; lthebaul@caltech.edu;
s1613153@sms.ed.ac.uk

date: 2019.05.30

updated: 2019.05.30
%}

%% NOTES

%{
- a version 2.0 of this code exists which includes tracking of a
time-dependent gaol
- all arguments to 'main' have been hard-coded to make the function trivial
to run
%}

%% CODE

function[] = main() % main(r,x_c,x_d,T,delta_t,K,alpha);

clear; clc;

% profile on;

% Arguments to 'main' - hard-coded for simplicity
r = 1; x_c = [-10;-15]; x_d = [15;15];
T = 2000; delta_t = 0.01;
K = 0.01; alpha = 5;

% CPU time registered to determine total computation time
cpu_t           = cputime;

% Call 'workspace' function which defines the workspace and obstacles
[W_x,W_y,x_obs,r_obs]   = workspace();

% Estimated projected goal initial position 'x_c' as initial condition
x_hat           = x_c;

% Creation of matrices used to log the robot position and estimated
% projected goal over time
x_c_cat         = [];
x_hat_cat       = [];

% Solution of ODEs through basic implementation of Euler's method with
% constant time-steps
for t = 1:delta_t:T
    x_c_cat     = cat(2,x_c_cat,x_c);
    x_hat_cat   = cat(2,x_hat_cat,x_hat);
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
nav_plot(W_x,W_y,x_obs,r_obs,x_c_cat,x_hat_cat);

% Display elapsed time during computation
disp(['Elasped time: ',num2str(cputime - cpu_t)]);

% profile off;
% profile viewer;

end
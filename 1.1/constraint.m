%% constraint

%% DESCRIPTION

%{
name: constraint

type: function

input: 'x_obs', 'r_obs' obstacle positions, radii; 'r', 'x_c', 'x_d' robot
radius, initial position; 'x_hat' estimation of projected goal; 'K'
controller gain

output: 'a', 'b', 'a_dot', 'b_dot' miscellaneous constraint parameters

purpose: this function computes the constraint parameters from relevant
inputs

author:     Leopold Thebault, leopold@thebault.lt; lthebaul@caltech.edu;
s1613153@sms.ed.ac.uk

date: 2019.05.30

updated: 2019.05.30
%}

%% NOTES

%{
- 'theta', 'b', 'theta_dot', 'b_dot' are most computationally expensive
%}

%% CODE

function[a,b,a_dot,b_dot] = ...
    constraint(x_obs,r_obs,x_c,r,x_hat,K)

% Number of obstacles
m               = size(x_obs,2);

% Constraint parameters
a               = x_obs - x_c;
theta           = .5 - (r_obs.^2 - r^2)./(2*vecnorm((x_obs - x_c),2,1).^2);
                % .5 - (r_obs.^2 - r^2)./(2*sum((x_obs - x_c).^2,1));
b               = diag((x_obs - x_c).'*(theta.*x_obs + (1 - ...
    theta).*x_c + r.*(x_c - x_obs)./vecnorm((x_c - x_obs),2,1)));

% First derivatives of constraints
x_c_dot         = -K.*(x_c - x_hat);
a_dot           = repmat(-x_c_dot,1,m);
theta_dot       = diag(K.*(x_c - x_obs).'*(x_c - ...
    x_hat)./(vecnorm((x_c - x_obs),2,1).^4)).';
b_dot           = (-x_c_dot.'*(theta.*x_obs + (1 - theta).*x_c) + ...
    theta_dot.*vecnorm((x_obs - x_c),2,1).^2.*(r - theta)*(x_obs - ...
    x_c).'*x_c_dot).';

end
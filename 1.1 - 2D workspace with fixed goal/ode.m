%% ode

%% DESCRIPTION

%{
name: ode

type: function

input: 'x' or 'x_hat, 'x_d' estimated projected goal, destination/goal; 't'
ODE computed time; 'a', 'b', 'a_dot', 'b_dot' miscellaneous constraint
parameters; 'alpha' correction term coefficient; 'x_obs' obstacle positions

output: 'x_dot' ODE definition of estimated projected goal's first
derivative

purpose: this function implements the full ODE from (23) in the
aforementioned paper (cf main) which defines the estimated projected goal's
first derivative

author:     Leopold Thebault, leopold@thebault.lt; lthebaul@caltech.edu;
s1613153@sms.ed.ac.uk

date: 2019.05.30

updated: 2019.05.30
%}

%% NOTES

%{
- matrix multiplication in 'del_xx_phi' and inversion in 'x_dot' are most
computationally expensive
%}

%% CODE

function[x_dot] = ode(x,x_d,t,a,b,a_dot,b_dot,alpha,x_obs)

% Number of dimensions in space and obstacles
[n,m]           = size(x_obs);

% Correction factor P
P               = alpha.*eye(n);

% Barrier parameter and its derivative
c               = exp(.001*t);
c_dot           = .001*exp(.001*t);


% Computation of ODE terms

% Common term
del_xx_phi_num  = bsxfun(@times,reshape(a,n,1,m),reshape(a,1,n,m));
del_xx_phi_den  = reshape((b - a.'*x).^2,1,1,m);
del_xx_phi      = eye(n) + 1/c.*sum(del_xx_phi_num./del_xx_phi_den,3);
% Prediction component
del_xt_phi      = -c_dot/(c^2).*sum( (a./((b - a.'*x).')) ,2) + ...
    1/c.*sum(a_dot./((b - a.'*x).'),2) - ...
    1/c.*sum(a.*(((b_dot - a_dot.'*x)./((b - a.'*x).^2)).'),2);
% Correction component
del_x_phi       = x - x_d + 1/c.*sum( (a./((b - a.'*x).')) ,2);

% Full ODE prediction-correction ODE
x_dot           = -inv(del_xx_phi)*(P*del_x_phi + del_xt_phi);

end

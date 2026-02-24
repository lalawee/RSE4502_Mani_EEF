function path = defineWeldPath(varargin)
% defineWeldPath
% Defines the circular welding path in the robot base frame.
%
% Optional Name-Value inputs:
%   'omega'      - angular velocity (rad/s), default 0.5
%   'dt'         - timestep (s), default 0.05
%   'baseOffset' - robot base X offset from cylinder center (m), default 0.8
%
% Output: path struct with fields
%   t          - 1xN time vector
%   pos        - 3xN position [x;y;z] in robot base frame
%   vel        - 3xN linear velocity [xdot;ydot;zdot]
%   R_tool     - 3x3 fixed tool orientation
%   omega_tool - 3x1 angular velocity (zero)
%   v_des      - 6xN desired task space velocity [angular; linear]
%   N          - number of timesteps

p = inputParser;
addParameter(p, 'omega',      0.5);
addParameter(p, 'dt',         0.05);
addParameter(p, 'baseOffset', 0.8);
parse(p, varargin{:});

omega      = p.Results.omega;
dt         = p.Results.dt;
base_x     = p.Results.baseOffset;

% Geometry
r_weld = 0.5;
z_weld = 1.0;

% Time vector
T_total = 2*pi / omega;
t       = 0 : dt : T_total;
N       = length(t);

% Position in robot base frame
x = r_weld * cos(omega * t) - base_x;
y = r_weld * sin(omega * t);
z = z_weld * ones(1, N);

% Linear velocity
xdot = -r_weld * omega * sin(omega * t);
ydot =  r_weld * omega * cos(omega * t);
zdot =  zeros(1, N);

% Tool orientation: fixed, pointing straight down
R_tool     = eul2rotm([0, pi, 0], 'ZYX');
omega_tool = [0; 0; 0];

% Task space velocity [angular; linear]
v_des = [repmat(omega_tool, 1, N); xdot; ydot; zdot];

% Pack output
path.t          = t;
path.pos        = [x; y; z];
path.vel        = [xdot; ydot; zdot];
path.R_tool     = R_tool;
path.omega_tool = omega_tool;
path.v_des      = v_des;
path.N          = N;
path.omega      = omega;
path.dt         = dt;
path.base_x     = base_x;

fprintf('Welding path defined:\n');
fprintf('  Seam radius  : %.2f m\n', r_weld);
fprintf('  Seam height  : %.2f m\n', z_weld);
fprintf('  Angular vel  : %.2f rad/s\n', omega);
fprintf('  Base offset  : %.2f m\n', base_x);
fprintf('  Timesteps    : %d (dt=%.3fs)\n', N, dt);
fprintf('  EE X range   : [%.3f, %.3f] m (robot base frame)\n', min(x), max(x));
fprintf('  EE Y range   : [%.3f, %.3f] m (robot base frame)\n\n', min(y), max(y));

end
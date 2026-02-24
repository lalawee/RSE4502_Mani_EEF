function path = defineWeldPath(varargin)
% defineWeldPath
% Defines the circular welding path in the robot base frame.
%
% Robot base is at (1.0, 0, 0) in world frame.
% Cylinder center is at world origin (0, 0, 0).
% Seam: radius=0.5m, height=1.0m around cylinder.
%
% Optional Name-Value inputs:
%   'omega'   - angular velocity (rad/s), default 0.5
%   'dt'      - timestep (s), default 0.05
%
% Output: path struct with fields
%   t         - 1xN time vector
%   pos       - 3xN position [x;y;z] in robot base frame
%   vel       - 3xN linear velocity [xdot;ydot;zdot]
%   R_tool    - 3x3 fixed tool orientation (constant)
%   omega_tool - 3x1 angular velocity (zero, orientation fixed)
%   v_des     - 6xN desired task space velocity [angular; linear]
%   N         - number of timesteps

% --- Parameters ---
p = inputParser;
addParameter(p, 'omega', 0.5);   % rad/s
addParameter(p, 'dt',    0.05);  % s
parse(p, varargin{:});

omega  = p.Results.omega;
dt     = p.Results.dt;

% --- Geometry ---
r_weld    = 0.5;    % seam radius (m) in world frame
z_weld    = 1.0;    % seam height (m)
base_x    = 0.8;    % robot base X offset from cylinder center (m)

% --- Time vector (one full revolution) ---
T_total = 2*pi / omega;
t       = 0 : dt : T_total;
N       = length(t);

% --- Position in robot base frame ---
% World frame seam: [0.5*cos, 0.5*sin, 1.0]
% Robot base at (1.0, 0, 0) => subtract base offset in X
x = r_weld * cos(omega * t) - base_x;
y = r_weld * sin(omega * t);
z = z_weld * ones(1, N);

% --- Linear velocity (analytical derivative) ---
xdot = -r_weld * omega * sin(omega * t);
ydot =  r_weld * omega * cos(omega * t);
zdot =  zeros(1, N);

% --- Tool orientation: fixed, pointing straight down ---
% RotY(180deg) => Z-axis of tool points in -Z world direction
R_tool     = eul2rotm([0, pi, 0], 'ZYX');
omega_tool = [0; 0; 0];   % no angular velocity

% --- Task space velocity vector [angular; linear] ---
v_des = [repmat(omega_tool, 1, N);   % 3xN angular (zeros)
         xdot; ydot; zdot];          % 3xN linear

% --- Pack output ---
path.t          = t;
path.pos        = [x; y; z];
path.vel        = [xdot; ydot; zdot];
path.R_tool     = R_tool;
path.omega_tool = omega_tool;
path.v_des      = v_des;
path.N          = N;
path.omega      = omega;
path.dt         = dt;

fprintf('Welding path defined:\n');
fprintf('  Seam radius : %.2f m\n', r_weld);
fprintf('  Seam height : %.2f m\n', z_weld);
fprintf('  Angular vel : %.2f rad/s\n', omega);
fprintf('  Timesteps   : %d (dt=%.3fs)\n', N, dt);
fprintf('  EE X range  : [%.3f, %.3f] m (robot base frame)\n', min(x), max(x));
fprintf('  EE Y range  : [%.3f, %.3f] m (robot base frame)\n', min(y), max(y));

end
function [R_wc, camPos_world, used, resnorm] = estimatePoseFixedZ(worldPoints, imagePoints, intr, z0, initialGuess)
% ESTIMATEPOSEFIXEDZ  Estimate camera pose with fixed camera height z=z0.
%   worldPoints: Nx3 world coords
%   imagePoints: Nx2 pixel coords (undistorted)
%   intr: cameraIntrinsics
%   z0: fixed camera height (meters)
%   initialGuess: optional struct with fields R_wc (3x3), camPos (1x3)
%
% Returns:
%   R_wc: world->camera rotation
%   camPos_world: 1x3 camera position in world coords
%   used: method name
%   resnorm: final residual norm

worldPoints = double(worldPoints);
imagePoints = double(imagePoints);

N = size(worldPoints,1);
if size(imagePoints,1) ~= N
    error('Mismatch between worldPoints and imagePoints sizes.');
end

% Build K
K = intr.IntrinsicMatrix';
fx = K(1,1); fy = K(2,2); cx = K(1,3); cy = K(2,3);

% ----- initialize parameters -----
if nargin >= 5 && ~isempty(initialGuess)
    if isfield(initialGuess,'R_wc') && isfield(initialGuess,'camPos')
        R0 = initialGuess.R_wc;
        C0 = initialGuess.camPos(:);      % camera pos in world
        t0 = -R0 * C0;                    % world->cam translation
        r0 = rotationMatrixToVector(R0);
        p0 = [r0; t0(1); t0(2)];          % params = [rvec(3), tx, ty]
    else
        p0 = zeros(5,1);
    end
else
    p0 = zeros(5,1);
end

% residual function
fun = @(p) residual_func(p, worldPoints, imagePoints, K, z0);

% choose solver
if exist('lsqnonlin','file')
    opts = optimoptions('lsqnonlin','Display','off','MaxFunctionEvaluations',2000);
    [p_opt,resnorm] = lsqnonlin(fun, p0, [], [], opts);
    used = 'fixedZ-lsqnonlin';
else
    % fallback fminunc
    costfun = @(p) sum(fun(p).^2);
    opts = optimoptions('fminunc','Display','off','MaxFunctionEvaluations',2000);
    [p_opt, resnorm] = fminunc(costfun, p0, opts);
    used = 'fixedZ-fminunc';
end

% unpack
rvec = p_opt(1:3);
tx   = p_opt(4);
ty   = p_opt(5);

% compute R
R_wc = rotationVectorToMatrix(rvec);

% compute tz such that camPos_world(3) = z0
% from: C = -R_wc' * t
%      C_z = z0  =>  -(R_wc'(3,:) * [tx;ty;tz]) = z0
r31 = R_wc(3,1);
r32 = R_wc(3,2);
r33 = R_wc(3,3);

tz = -(z0 + r31*tx + r32*ty) / r33;

t = [tx; ty; tz];     % translation world->cam

% camera world position
camPos_world = (-R_wc' * t).';

end

%% ============ helper residual ===================
function r = residual_func(p, worldPoints, imagePoints, K, z0)
rvec = p(1:3);
tx   = p(4);
ty   = p(5);

R_wc = rotationVectorToMatrix(rvec);

r31 = R_wc(3,1);
r32 = R_wc(3,2);
r33 = R_wc(3,3);

tz = -(z0 + r31*tx + r32*ty) / r33;
t  = [tx; ty; tz];

N = size(worldPoints,1);
Xh = [worldPoints, ones(N,1)]';

P = K * [R_wc, t];
proj_h = P * Xh;
proj = proj_h(1:2,:) ./ proj_h(3,:);

res = proj' - imagePoints;
r = res(:);
end

%% ============ Rodrigues helpers =================
function R = rotationVectorToMatrix(r)
theta = norm(r);
if theta < 1e-12
    R = eye(3);
    return;
end
k = r/theta;
K = [  0   -k(3)  k(2);
      k(3)   0   -k(1);
     -k(2) k(1)    0];
R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
end

function r = rotationMatrixToVector(R)
angle = acos(max(-1,min(1,(trace(R)-1)/2)));
if abs(angle) < 1e-12
    r = zeros(3,1);
    return;
end
rx = (R(3,2)-R(2,3))/(2*sin(angle));
ry = (R(1,3)-R(3,1))/(2*sin(angle));
rz = (R(2,1)-R(1,2))/(2*sin(angle));
r = angle * [rx; ry; rz];
end
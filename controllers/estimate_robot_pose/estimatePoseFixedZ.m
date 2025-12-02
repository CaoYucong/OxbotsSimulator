function [R_wc, camPos_world, used, resnorm] = estimatePoseFixedZ(worldPoints, imagePoints, intr, z0, initialGuess)
% ESTIMATEPOSEFIXEDZ Estimate camera pose with fixed camera height z = z0.
%   worldPoints: Mx3 world coords (meters)
%   imagePoints: Mx2 image pixel coords (pixels)
%   intr: cameraIntrinsics object
%   z0: scalar, camera height in world coordinates (meters)
%   initialGuess: optional struct with fields R_wc (3x3 world->cam) and camPos (1x3)
%
% Returns:
%   R_wc: 3x3 rotation matrix world->camera (so X_cam = R_wc * X_w + t)
%   camPos_world: 1x3 camera position in world coords (C)
%   used: string tag
%   resnorm: final sum of squared residuals

% Dependencies: Optimization Toolbox (lsqnonlin). If unavailable, fallback to fminunc (slower).

if nargin < 5
    initialGuess = [];
end

% Ensure sizes
worldPoints = double(worldPoints);
imagePoints = double(imagePoints);
M = size(worldPoints,1);
if size(imagePoints,1) ~= M
    error('worldPoints and imagePoints must have same number of points');
end

% Build K (standard convention)
K = intr.IntrinsicMatrix'; % 3x3

% initial guess for parameters: r (3), tx, ty (2)
% If user provided extrinsics-like initial, use them
if ~isempty(initialGuess) && isfield(initialGuess,'R_wc') && isfield(initialGuess,'camPos')
    R0 = initialGuess.R_wc;
    C0 = reshape(initialGuess.camPos,3,1);
    t0 = -R0 * C0; % t such that X_cam = R_wc * X_w + t
    % convert R0 to Rodrigues
    r0 = rotationMatrixToVector(R0); % 3x1
    p0 = [r0(:); t0(1); t0(2)]; % don't include t0(3)
else
    % fallback: look-at initialization: position camera at centroid + some offset
    centroid = mean(worldPoints,1)';
    % put camera at (centroid.x, centroid.y, z0) looking toward centroid
    C_guess = [centroid(1); centroid(2); z0];
    zc = (centroid - C_guess); zc = zc / norm(zc);
    % create R so that camera z-axis points forward = (0,0,1) in camera coords maps to -zc in world?
    % Simpler: use extrinsics from extrinsics() with synthetic small guess if possible
    % We'll use identity rotation as fallback
    R0 = eye(3);
    t0 = -R0 * C_guess;
    r0 = [0;0;0];
    p0 = [r0; t0(1); t0(2)];
end

% residual function: params p = [r1 r2 r3 tx ty] (r is Rodrigues for R_wc)
fun = @(p) reproj_residuals_fixedz(p, worldPoints, imagePoints, K, z0);

% choose solver
useLSQ = exist('lsqnonlin','file') == 2;
opts = [];
if useLSQ
    opts = optimoptions('lsqnonlin','Display','off','MaxFunctionEvaluations',2000,'FunctionTolerance',1e-8);
    [p_opt,resnorm,~,exitflag] = lsqnonlin(fun, p0, [], [], opts);
else
    % fallback to fminunc (minimize sum squares)
    fprintf('Warning: lsqnonlin not found. Using fminunc fallback (slower).\n');
    if exist('fminunc','file')~=2
        error('No suitable optimizer found (need lsqnonlin or fminunc).');
    end
    opts = optimoptions('fminunc','Display','off','MaxFunctionEvaluations',2000,'StepTolerance',1e-8);
    costfun = @(p) sum(fun(p).^2);
    [p_opt, fval, exitflag] = fminunc(costfun, p0, opts);
    resnorm = fval;
end

% unpack optimal params
r_opt = p_opt(1:3);
tx_opt = p_opt(4);
ty_opt = p_opt(5);
R_wc = rotationVectorToMatrix(r_opt(:));   % world->cam
t_vec = [tx_opt; ty_opt; - (R_wc * reshape([0;0;z0],3,1))(3)]; %#ok<NASGU> % t computed below properly

% derive camera position in world coords:
% We have X_cam = R_wc * X_w + t  => when X_w = C (camera pos) then X_cam = 0 => 0 = R_wc * C + t => C = -R_wc^{-1} * t = -R_wc' * t
% We computed t = [tx,ty,tz] where tz is set so that camera pos z == z0. Solve for tz:
% Let t = [tx;ty;tz], we want C_z = z0. From C = -R_wc' * t, C_z = - (R_wc' * t)(3) = z0 -> gives linear eq for tz.
% Solve for tz:
tx = tx_opt; ty = ty_opt;
% compute tz by solving - (R_wc' * [tx;ty;tz])_3 = z0  => -(r31*tx + r32*ty + r33*tz) = z0
r31 = R_wc(3,1); r32 = R_wc(3,2); r33 = R_wc(3,3);
% - (r31*tx + r32*ty + r33*tz) = z0  => tz = -( z0 + r31*tx + r32*ty ) / r33
tz = -( z0 + r31*tx + r32*ty ) / r33;
t_final = [tx; ty; tz];

% camera position in world:
C = -R_wc' * t_final;
camPos_world = C(:)';

used = 'fixedZ-lsq';
end

%% ---------------- helper: residuals ----------------
function r = reproj_residuals_fixedz(p, worldPoints, imagePoints, K, z0)
    % p: 5x1 [rvec(3); tx; ty]
    rvec = p(1:3);
    tx = p(4); ty = p(5);
    R_wc = rotationVectorToMatrix(rvec(:));
    % compute tz so that camera world-z == z0 (same math as above)
    r31 = R_wc(3,1); r32 = R_wc(3,2); r33 = R_wc(3,3);
    tz = -( z0 + r31*tx + r32*ty ) / r33;
    t = [tx; ty; tz]; % 3x1 translation such that X_cam = R_wc * X_w + t
    % project points
    M = size(worldPoints,1);
    Xh = [worldPoints, ones(M,1)]';
    P = K * [R_wc, t];
    proj_h = P * Xh;
    proj2 = bsxfun(@rdivide, proj_h(1:2,:), proj_h(3,:))';
    % residuals stacked (2M x 1)
    rvecs = proj2 - imagePoints;
    r = rvecs(:);
end

%% ---------------- helper: Rodrigues/rotation helpers ----------------
function R = rotationVectorToMatrix(r)
    % r: 3x1 Rodrigues vector (axis * angle)
    theta = norm(r);
    if theta < 1e-12
        R = eye(3);
        return;
    end
    k = r / theta;
    K = [   0   -k(3)  k(2);
          k(3)   0   -k(1);
         -k(2) k(1)    0];
    R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
end

function r = rotationMatrixToVector(R)
    % convert rotation matrix to Rodrigues vector (axis*angle)
    angle = acos( max(-1,min(1, (trace(R)-1)/2 )) );
    if abs(angle) < 1e-12
        r = zeros(3,1);
        return;
    end
    rx = (R(3,2)-R(2,3))/(2*sin(angle));
    ry = (R(1,3)-R(3,1))/(2*sin(angle));
    rz = (R(2,1)-R(1,2))/(2*sin(angle));
    r = angle * [rx; ry; rz];
end
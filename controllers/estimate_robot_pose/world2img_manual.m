function imagePoints = world2img_manual(worldPoints, tform, intr, applyDistortion)
% WORLD2IMG_MANUAL Project world points to image pixels (manual implementation).
%   imagePoints = world2img_manual(worldPoints, tform, intr)
%   imagePoints = world2img_manual(worldPoints, tform, intr, applyDistortion)
%
% Inputs:
%   worldPoints      - Nx3 matrix of world coordinates (double)
%   tform            - rigidtform3d object representing camera pose:
%                      tform.R is 3x3 camera->world rotation (R_c2w)
%                      tform.Translation is 1x3 camera position in world (cam_pos)
%   intr             - cameraIntrinsics object
%   applyDistortion  - optional bool (default true). If true, apply radial/tangential distortion.
%
% Output:
%   imagePoints      - Nx2 pixel coordinates (u,v)
%
% Conventions:
%   We use X_cam = R_wc * X_world + t   (world->camera)
%   where R_wc = (R_c2w)' and t = -R_wc * cam_pos'
%
% Example:
%   tform = rigidtform3d(R_c2w, cam_pos);
%   uv = world2img_manual(worldCorners, tform, intr, true);

if nargin < 4
    applyDistortion = true;
end

% validate input shapes
if size(worldPoints,2) ~= 3
    error('worldPoints must be N x 3');
end
worldPoints = double(worldPoints);
N = size(worldPoints,1);

% extract R_c2w and cam_pos from tform
if ~isa(tform,'rigidtform3d')
    error('tform must be a rigidtform3d object');
end
R_c2w = double(tform.R);               % 3x3 camera->world
cam_pos = double(tform.Translation(:)); % 3x1 camera position in world

% compute world -> camera rotation and translation (MATLAB convention)
R_wc = R_c2w.';            % world -> camera
t = - R_wc * cam_pos;     % 3x1, so that X_cam = R_wc * X_world + t

% camera intrinsics: we support cameraIntrinsics fields
% intr.IntrinsicMatrix (3x3 column-major), intr.FocalLength [fx fy], intr.PrincipalPoint [cx cy]
if isprop(intr,'IntrinsicMatrix')
    K = intr.IntrinsicMatrix'; % convert to conventional K (row-major)
else
    error('intr must be a cameraIntrinsics object with IntrinsicMatrix property');
end

% optional distortion params
k_rad = [0,0,0]; % k1 k2 k3
p_tan = [0,0];   % p1 p2
if applyDistortion
    if isprop(intr,'RadialDistortion')
        rd = intr.RadialDistortion;
        % rad can be length 0,2,3; copy into k_rad
        k_rad(1:numel(rd)) = rd(:)';
    end
    if isprop(intr,'TangentialDistortion')
        td = intr.TangentialDistortion;
        if numel(td)>=2
            p_tan = td(1:2);
        end
    end
end

% project each world point
% transform to camera coords
Xc = (R_wc * worldPoints.' + repmat(t,1,N))'; % N x 3

% avoid points behind camera (Z <= 0) — still project but warn
if any(Xc(:,3) <= 0)
    warning('Some points have non-positive Z in camera frame (behind camera or on plane). Results may be invalid.');
end

% normalized image plane coords (pinhole)
x = Xc(:,1) ./ Xc(:,3);   % N x 1
y = Xc(:,2) ./ Xc(:,3);

% apply distortion if requested
if applyDistortion && (any(k_rad~=0) || any(p_tan~=0))
    r2 = x.^2 + y.^2;
    % radial: k1,k2,k3
    k1 = k_rad(1); k2 = k_rad(2); k3 = k_rad(3);
    radial = 1 + k1.*r2 + k2.*(r2.^2) + k3.*(r2.^3);
    % tangential:
    p1 = p_tan(1); p2 = p_tan(2);
    x_t = 2*p1.*x.*y + p2.*(r2 + 2*x.^2);
    y_t = p1.*(r2 + 2*y.^2) + 2*p2.*x.*y;
    xd = x .* radial + x_t;
    yd = y .* radial + y_t;
else
    xd = x;
    yd = y;
end

% map to pixel coordinates using K
% K = [fx s cx; 0 fy cy; 0 0 1] assumed
fx = K(1,1);
s  = K(1,2);
cx = K(1,3);
fy = K(2,2);
cy = K(2,3);

u = fx .* xd + s .* yd + cx;
v = fy .* yd + cy;

imagePoints = [u, v];

end
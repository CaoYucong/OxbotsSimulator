function results = estimate_world_coordinates_from_all_apriltag(intrStruct, temp_img_path)
% ESTIMATE_WORLD_COORDINATES_FROM_ALL_APRILTAG Aggregate all detected tag corners and solve PnP once.
%   results = estimate_world_coordinates_from_all_apriltag(intrStruct, temp_img_path)
%
% intrStruct: cameraIntrinsics object (optional, pass [] to load from disk)
% temp_img_path: image path (optional, default '../../cache/camera/temp_img.png')
%
% returns results: structure array per tag (same fields as previous per-tag function)

if nargin < 2 || isempty(temp_img_path)
    temp_img_path = "../../cache/camera/temp_img.png";
end

% ---- create intr (cameraIntrinsics) and cameraParams (cameraParameters) from input ----
[intr, cameraParams] = build_intrinsics_from_input(intrStruct);


% read image + detect
I = imread(temp_img_path);
tagFamily = "tag36h11";
[ids, loc, detectedFamily] = readAprilTag(I, tagFamily);
if isempty(ids)
    error('No AprilTags detected.');
end
if size(loc,3) == 1
    loc = reshape(loc, size(loc,1), size(loc,2), 1);
end

% draw detection messages (no heavy drawing)
for idx = 1:length(ids)
    disp("Detected Tag ID, Family: " + ids(idx) + ", " + detectedFamily(idx));
end

% save a copy
[folder, ~, ~] = fileparts(temp_img_path);
output_path = fullfile(folder, "temp_img_recognised.png");
imwrite(I, output_path);

% normalize ids / loc
ids = double(ids(:));
N = numel(ids);
if N == 0, error('ids empty'); end

if isequal(size(loc), [4,2,N])
    loc3 = loc;
elseif ismatrix(loc) && size(loc,1) == N && size(loc,2) == 8
    loc3 = zeros(4,2,N);
    for i=1:N
        row = loc(i,:);
        loc3(:,:,i) = reshape(row,2,4)';
    end
elseif isvector(loc) && numel(loc) == 8*N
    tmp = reshape(loc, [2,4,N]);
    loc3 = permute(tmp, [2,1,3]);
else
    error('loc must be 4x2xN or N x 8 or vector length 8N. Got size %s', mat2str(size(loc)));
end

% load tag world map
tagWorldMap = load_tagWorldMap();

% build id->worldCorners map
mapById = containers.Map('KeyType','double','ValueType','any');
for k = 1:numel(tagWorldMap)
    mapById(double(tagWorldMap(k).id)) = tagWorldMap(k).worldCorners; % 4x3
end

% aggregate all detected corners
allImgPts = zeros(0,2);
allWorldPts = zeros(0,3);
tagPointIdx = zeros(0,1); % which tag each point came from
cornerIdx = zeros(0,1);   % corner index 1..4

for k = 1:N
    id = ids(k);
    ci = squeeze(loc3(:,:,k)); % 4x2
    if any(isnan(ci(:)))
        warning('Tag %d: some corners NaN, skipping those corners', id);
    end
    if ~isKey(mapById, id)
        warning('Tag %d not in tagWorldMap, skipping tag', id);
        continue;
    end
    worldCorners = mapById(id); % 4x3 (BL,BR,TR,TL)
    % undistort points (cameraParams assumed no distortion for theoretical intr)
    try
        ptsUnd = undistortPoints(ci, cameraParams); % 4x2
    catch
        ptsUnd = ci;
    end
    % append
    startIdx = size(allImgPts,1) + 1;
    allImgPts = [allImgPts; ptsUnd];
    allWorldPts = [allWorldPts; worldCorners];
    tagPointIdx = [tagPointIdx; repmat(k,4,1)];
    cornerIdx = [cornerIdx; (1:4)'];
end

if isempty(allImgPts)
    error('No valid image points aggregated.');
end

% try estworldpose on aggregated points (preferred)
usedMethod = '';
orientation = []; location = [];
hasEstWorldPoseNew = exist('estworldpose','file') == 2;
try
    if hasEstWorldPoseNew
        [worldPose, inlierIdx, status] = estworldpose(allImgPts, allWorldPts, intr, ...
            MaxReprojectionError=4, Confidence=95, MaxNumTrials=500);
        if status == 0
            R_c2w = worldPose.R; cam_pos = worldPose.Translation;
            orientation = R_c2w'; % world->cam
            location = cam_pos;
            usedMethod = 'estworldpose(combined)';
        else
            warning('estworldpose(combined) returned status %d; will try extrinsics fallback.', status);
        end
    end
catch ME
    warning('estworldpose(combined) failed: %s', ME.message);
end

% fallback to extrinsics on combined points
if isempty(orientation)
    try
        [R_ex_all, t_ex_all] = extrinsics(allImgPts, allWorldPts, cameraParams);
        orientation = R_ex_all; % world->cam
        location = (-orientation' * t_ex_all')';
        usedMethod = 'extrinsics(combined)';
    catch ME
        error('Combined extrinsics failed: %s', ME.message);
    end
end

% compute T and reprojection for all points
R_c2w = orientation'; % camera->world
cam_pos = location(:)'; % 1x3
T_world_from_cam = eye(4);
T_world_from_cam(1:3,1:3) = R_c2w;
T_world_from_cam(1:3,4) = cam_pos';

tform = rigidtform3d(R_c2w, cam_pos);
projAll = world2img_manual(allWorldPts, tform, intr, true);

reprojErrAll = sqrt(sum((projAll - allImgPts).^2, 2));
overallMeanReproj = mean(reprojErrAll);

% build per-tag results (reuse measured corners order from loc3)
results = struct([]);
uniqueTags = unique(tagPointIdx);
for kk = 1:numel(uniqueTags)
    tagIdx = uniqueTags(kk);
    id = ids(tagIdx);
    if ~isKey(mapById, id), continue; end
    worldCorners = mapById(id); % 4x3
    corners_img = squeeze(loc3(:,:,tagIdx)); % 4x2
    % find indices in aggregated arrays
    idxs = find(tagPointIdx == tagIdx);
    proj_tag = projAll(idxs, :);
    reproj_tag = reprojErrAll(idxs);
    meanReproj_tag = mean(reproj_tag);
    % orientation/location refer to combined solution
    R_c2w_tag = R_c2w;
    cam_pos_tag = cam_pos;
    T_world_from_cam_tag = T_world_from_cam;
    quat = rotm2quat(R_c2w_tag);
    eulZYX = rotm2eul(R_c2w_tag,'ZYX');
    res.id = id;
    res.corners_img = corners_img;
    % find undistorted corners from allImgPts entries
    res.corners_undistorted = allImgPts(idxs,:);
    res.worldCorners = worldCorners;
    res.method = usedMethod;
    res.orientation = orientation;
    res.location = cam_pos_tag;
    res.T_world_from_cam = T_world_from_cam_tag;
    res.cam_position_world = cam_pos_tag;
    res.quat_wxyz = quat;
    res.eulZYX_deg = rad2deg(eulZYX);
    res.reprojPerCorner = reproj_tag;
    res.meanReproj = meanReproj_tag;
    results = [results; res]; %#ok<AGROW>
end

% annotate image: measured red, projected green (use first tag's corners_img/world proj as debug)
% draw all measured (red) and all projected (green)
if ~isempty(allImgPts)
    I = insertShape(I, "Circle", [allImgPts, repmat(4,size(allImgPts,1),1)], 'ShapeColor','red','Opacity',1);
    I = insertShape(I, "Circle", [projAll, repmat(4,size(projAll,1),1)], 'ShapeColor','green','Opacity',1);
end

figure; imshow(I); hold off;
fprintf('Combined-solve using %s. Total points=%d. Overall mean reproj = %.3f px\n', usedMethod, size(allImgPts,1), overallMeanReproj);

% print brief summary
fprintf('Per-tag (from combined solution) results: %d tags.\n', numel(results));
for i=1:numel(results)
    fprintf(' id=%2d  meanReproj=%.3f px  camPos=[%.3f %.3f %.3f]\n', ...
        results(i).id, results(i).meanReproj, results(i).cam_position_world);
end

% show extrinsics
showExtrinsics(results, cameraParams);

end

%% ---------------- helper: build_intrinsics_from_input ----------------
function [intr, cameraParams] = build_intrinsics_from_input(inObj)
% Accept cameraIntrinsics, or struct with FocalLength/PrincipalPoint/ImageSize,
% or struct with field intr (cameraIntrinsics)
if isempty(inObj)
    error('intrStruct is empty. Pass cameraIntrinsics or struct with FocalLength/PrincipalPoint/ImageSize.');
end

% if it's a cameraIntrinsics already
if isa(inObj, 'cameraIntrinsics')
    intr = inObj;
else
    % if provided as a struct with field 'intr'
    if isstruct(inObj) && isfield(inObj, 'intr') && isa(inObj.intr, 'cameraIntrinsics')
        intr = inObj.intr;
    elseif isstruct(inObj) && all(isfield(inObj, {'FocalLength','PrincipalPoint','ImageSize'}))
        fx = inObj.FocalLength(1); fy = inObj.FocalLength(2);
        pp = inObj.PrincipalPoint;
        imgSz = inObj.ImageSize;
        intr = cameraIntrinsics([fx fy], pp, imgSz);
    else
        error('Unsupported intrStruct format. Provide cameraIntrinsics or struct with fields: FocalLength, PrincipalPoint, ImageSize.');
    end
end

% construct cameraParameters for functions that need it (assume zero distortion unless provided)
% if caller passed a cameraParameters inside struct as cameraParams, respect it
if isstruct(inObj) && isfield(inObj,'cameraParams') && isa(inObj.cameraParams,'cameraParameters')
    cameraParams = inObj.cameraParams;
else
    camMat = intr.IntrinsicMatrix;
    imageSize = intr.ImageSize;
    cameraParams = cameraParameters('IntrinsicMatrix', camMat, 'ImageSize', imageSize, 'RadialDistortion',[0 0], 'TangentialDistortion',[0 0]);
end
end

%% ---------------- helper: load_tagWorldMap ----------------
function tagWorldMap = load_tagWorldMap()
    candidates = { fullfile(pwd,'tagWorldMap.mat'), fullfile(pwd,'..','..','cache','camera','tagWorldMap.mat'), fullfile('..','cache','camera','tagWorldMap.mat') };
    for i=1:numel(candidates)
        f = candidates{i};
        if exist(f,'file')
            s = load(f);
            if isfield(s,'tagWorldMap'), tagWorldMap = s.tagWorldMap;
            else fn = fieldnames(s); tagWorldMap = s.(fn{1}); end
            return;
        end
    end
    error('tagWorldMap.mat not found. Generate it and place in candidate paths.');
end

%% ---------------- helper: world2img_manual (same as before) ----------------
function imagePoints = world2img_manual(worldPoints, tform, intr, applyDistortion)
    if nargin < 4, applyDistortion = true; end
    if size(worldPoints,2) ~= 3, error('worldPoints must be N x 3'); end
    worldPoints = double(worldPoints);
    N = size(worldPoints,1);
    if ~isa(tform,'rigidtform3d'), error('tform must be a rigidtform3d'); end
    R_c2w = double(tform.R);
    cam_pos = double(tform.Translation(:));
    R_wc = R_c2w.';
    t = - R_wc * cam_pos;
    if isprop(intr,'IntrinsicMatrix')
        K = intr.IntrinsicMatrix';
    else
        error('intr must be cameraIntrinsics with IntrinsicMatrix');
    end
    k_rad = [0,0,0];
    p_tan = [0,0];
    if applyDistortion
        if isprop(intr,'RadialDistortion'), rd = intr.RadialDistortion; k_rad(1:numel(rd)) = rd(:)'; end
        if isprop(intr,'TangentialDistortion'), td = intr.TangentialDistortion; if numel(td)>=2, p_tan = td(1:2); end; end
    end
    Xc = (R_wc * worldPoints.' + repmat(t,1,N))';
    if any(Xc(:,3) <= 0), warning('Some points behind camera (Z<=0).'); end
    x = Xc(:,1) ./ Xc(:,3);
    y = Xc(:,2) ./ Xc(:,3);
    if applyDistortion && (any(k_rad~=0) || any(p_tan~=0))
        r2 = x.^2 + y.^2;
        k1=k_rad(1); k2=k_rad(2); k3=k_rad(3);
        radial = 1 + k1.*r2 + k2.*(r2.^2) + k3.*(r2.^3);
        p1 = p_tan(1); p2 = p_tan(2);
        x_t = 2*p1.*x.*y + p2.*(r2 + 2*x.^2);
        y_t = p1.*(r2 + 2*y.^2) + 2*p2.*x.*y;
        xd = x .* radial + x_t;
        yd = y .* radial + y_t;
    else
        xd = x; yd = y;
    end
    fx = K(1,1); s = K(1,2); cx = K(1,3);
    fy = K(2,2); cy = K(2,3);
    u = fx .* xd + s .* yd + cx;
    v = fy .* yd + cy;
    imagePoints = [u, v];
end

%% ---------------- helper: showExtrinsics & drawCameraFrustum ----------------
function showExtrinsics(results, cameraParams)
    if nargin < 2, cameraParams = []; end
    if isempty(results), error('results empty'); end
    figure('Name','Per-tag Extrinsics (combined)','Color','w'); hold on; grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    for i=1:numel(results)
        wc = results(i).worldCorners;
        patch('XData', wc(:,1), 'YData', wc(:,2), 'ZData', wc(:,3), 'FaceColor', [0.9 0.9 0.9], 'FaceAlpha', 0.6);
        plot3([wc(:,1); wc(1,1)], [wc(:,2); wc(1,2)], [wc(:,3); wc(1,3)], '-k', 'LineWidth', 1);
        cen = mean(wc,1);
        text(cen(1), cen(2), cen(3), sprintf('id=%d', results(i).id), 'FontSize', 9, 'Color', 'k');
    end
    for i=1:numel(results)
        T = results(i).T_world_from_cam;
        drawCameraFrustum(T, cameraParams, 0.3, 0.1);
        pos = results(i).cam_position_world;
        text(pos(1), pos(2), pos(3), sprintf('id=%d e=%.2fpx', results(i).id, results(i).meanReproj), 'Color','b');
    end
    view(3); camorbit(20,-10);
    rotate3d on;
    hold off;
    title('Per-tag camera poses (combined-solution)');
end

function drawCameraFrustum(T_world_from_cam, cameraParams, depth, axisScale)
    if nargin<3, depth = 0.5; end
    if nargin<4, axisScale = 0.1; end
    if isprop(cameraParams,'Intrinsics')
        intr = cameraParams.Intrinsics;
    else
        intr = struct('ImageSize',[480,640],'FocalLength',[600,600],'PrincipalPoint',[320,240]);
    end
    imgSize = intr.ImageSize; W = imgSize(2); H = imgSize(1);
    fx = intr.FocalLength(1); fy = intr.FocalLength(2);
    cx = intr.PrincipalPoint(1); cy = intr.PrincipalPoint(2);
    imgPts = [1 1; W 1; W H; 1 H];
    camPts = zeros(4,3);
    for k=1:4
        u = imgPts(k,1); v = imgPts(k,2);
        x = (u - cx) / fx; y = (v - cy) / fy;
        camPts(k,:) = [x*depth, y*depth, depth];
    end
    P = [camPts'; ones(1,4)];
    Pw = (T_world_from_cam * P)'; originw = (T_world_from_cam * [0 0 0 1]')'; originw = originw(1:3);
    plot3([originw(1); Pw(:,1)], [originw(2); Pw(:,2)], [originw(3); Pw(:,3)], '-r');
    patch(Pw(:,1), Pw(:,2), Pw(:,3), 'r', 'FaceAlpha', 0.05, 'EdgeColor', 'r');
    R = T_world_from_cam(1:3,1:3);
    xs = originw + axisScale*(R(:,1)'); ys = originw + axisScale*(R(:,2)'); zs = originw + axisScale*(R(:,3)');
    plot3([originw(1) xs(1)], [originw(2) xs(2)], [originw(3) xs(3)], '-','Color',[0.8 0 0],'LineWidth',2);
    plot3([originw(1) ys(1)], [originw(2) ys(2)], [originw(3) ys(3)], '-','Color',[0 0.6 0],'LineWidth',2);
    plot3([originw(1) zs(1)], [originw(2) zs(2)], [originw(3) zs(3)], '-','Color',[0 0 0.8],'LineWidth',2);
end
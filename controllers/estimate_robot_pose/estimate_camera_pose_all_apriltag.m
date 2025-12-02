temp_img_path = "../../cache/camera/temp_img.png";

% ---- Read the input image ----
I = imread(temp_img_path);

% ---- AprilTag detection ----
tagFamily = "tag36h11";
[ids, loc, detectedFamily] = readAprilTag(I, tagFamily);

% ---- Draw markers on detected tag corners ----
for idx = 1:length(ids)
    disp("Detected Tag ID, Family: " + ids(idx) + ", " + detectedFamily(idx));
    
    markerRadius = 2;
    numCorners = size(loc, 1);
    markerPosition = [loc(:, :, idx), repmat(markerRadius, numCorners, 1)];
    
    % I = insertShape(I, "Circle", [loc(1, :, idx), 5], ...
    % ShapeColor="red", Opacity=1);
end

% ---- Build output path: same folder, fixed filename ----
[folder, ~, ~] = fileparts(temp_img_path);
output_path = fullfile(folder, "temp_img_recognised.png");

% ---- Save the processed image ----
imwrite(I, output_path);

disp("Detected " + length(ids) + " tags.\n");

% --- reshape/validate loc ---
ids = double(ids(:));
N = numel(ids);
if N == 0
    error('ids empty');
end

% normalize loc into 4x2xN
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

% --- load camera params and tagWorldMap ---
cameraParams = load_cameraParams();     % helper below
tagWorldMap = load_tagWorldMap();       % helper below

% build id->worldCorners map
mapById = containers.Map('KeyType','double','ValueType','any');
for k = 1:numel(tagWorldMap)
    mapById(double(tagWorldMap(k).id)) = tagWorldMap(k).worldCorners; % 4x3
end

% check availability of estworldpose
hasEstWorld = exist('estworldpose','file') == 2;

% ----------------- Combine all tag corners and solve once (estworldpose, no constraints) -----------------
allImgPts = [];    % Nx2 aggregated image points (pixel)
allWorldPts = [];  % Nx3 aggregated world points (meters)
tagPointIdx = [];  % which tag each point belongs to (1..N)
cornerIdx = [];    % which corner index (1..4) per point

for kk = 1:N
    id = ids(kk);
    if ~isKey(mapById, id)
        warning('Tag %d not in tagWorldMap, skipping', id);
        continue;
    end
    wc = mapById(id);            % 4x3 world corners (BL,BR,TR,TL)
    ci = squeeze(loc3(:,:,kk));  % 4x2 image corners
    % append in same order
    allImgPts = [allImgPts; ci];
    allWorldPts = [allWorldPts; wc];
    tagPointIdx = [tagPointIdx; repmat(kk,4,1)];
    cornerIdx = [cornerIdx; (1:4)'];
end

if isempty(allImgPts)
    error('No valid tag points to solve PnP.');
end

% undistort all image points (estworldpose expects undistorted image points)
try
    ptsUndAll = undistortPoints(allImgPts, cameraParams); % Mx2
catch
    ptsUndAll = allImgPts;
end

% --------- Use estworldpose (preferred) ----------
usedMethod = '';
orientation = []; location = []; inlierIdx = [];
if exist('estworldpose','file') == 2
    try
        % estworldpose requires undistorted image points and intrinsics
        [worldPose, inlierIdx, status] = estworldpose(ptsUndAll, allWorldPts, cameraParams.Intrinsics);
            % MaxReprojectionError=4, Confidence=99, MaxNumTrials=1000);
        if status == 0
            % worldPose.R is camera->world (rigidtform3d premultiply convention)
            R_c2w = worldPose.R;            % camera -> world
            cam_pos = worldPose.Translation; % 1x3
            % keep orientation as world->cam for compatibility with rest of script
            orientation = R_c2w';            % world -> cam
            location = cam_pos;              % camera location in world coords
            usedMethod = 'estworldpose';
        else
            warning('estworldpose returned status=%d (not enough inliers/points). Falling back to extrinsics.', status);
        end
    catch ME
        warning('estworldpose failed: %s. Falling back to extrinsics.', ME.message);
    end
end

% fallback to extrinsics on combined points if estworldpose failed or unavailable
if isempty(orientation)
    try
        [R_ex_all, t_ex_all] = extrinsics(ptsUndAll, allWorldPts, cameraParams);
        % extrinsics returns R_ex_all such that X_cam = R_ex_all * X_world' + t_ex_all'
        orientation = R_ex_all; % world->cam
        % compute camera location in world coords
        location = (-orientation' * t_ex_all')';
        usedMethod = 'extrinsics(combined)';
    catch ME
        error('Combined extrinsics failed: %s', ME.message);
    end
end

% Convert to camera-to-world pose (for visualization / further processing)
R_c2w = orientation';          % camera -> world rotation
cam_pos = location(:)';        % 1x3 camera position in world
T_world_from_cam = eye(4);
T_world_from_cam(1:3,1:3) = R_c2w;
T_world_from_cam(1:3,4) = cam_pos';

% compute extrinsics-like t_ex for reprojection: t_ex such that X_cam = orientation * X_world' + t_ex'
t_ex_for_proj = (-(orientation * cam_pos'))';  % 1x3

% reprojection for all points (diagnostics)
try
    projAll = worldToImage(cameraParams.Intrinsics, orientation, t_ex_for_proj, allWorldPts);
catch
    projAll = worldToImage(cameraParams, orientation, t_ex_for_proj, allWorldPts);
end
reprojErrAll = sqrt(sum((projAll - allImgPts).^2, 2)); % per-point px error

% annotate image I with world coords near each detected corner (optional)
strs = strings(size(allWorldPts,1),1);
for p = 1:size(allWorldPts,1)
    wc = allWorldPts(p,:);
    strs(p) = sprintf('(%.2f,%.2f,%.3f)', wc(1), wc(2), wc(3));
end
locsToAnnotate = ptsUndAll;
I = insertText(I, locsToAnnotate, strs, 'FontSize', 12, 'BoxColor', 'black', 'TextColor', 'white');

% Build results per-tag (fill with same fused pose, but compute per-tag reproj error & diagnostics)
results = struct([]);
for kk = 1:N
    id = ids(kk);
    if ~isKey(mapById, id)
        continue;
    end
    worldCorners = mapById(id);     % 4x3
    corners_img = squeeze(loc3(:,:,kk)); % 4x2
    % indices in aggregated arrays:
    idxs = find(tagPointIdx == kk);
    projPts_tag = projAll(idxs, :);
    reprojErrs_tag = reprojErrAll(idxs);
    meanReproj_tag = mean(reprojErrs_tag);
    
    res.id = id;
    res.corners_img = corners_img;
    res.corners_undistorted = reshape(ptsUndAll(idxs,:),(4),2);
    res.worldCorners = worldCorners;
    res.method = usedMethod;
    res.orientation = orientation; % world->cam
    res.location = cam_pos;        % camera position in world
    res.T_world_from_cam = T_world_from_cam;
    res.cam_position_world = cam_pos;
    res.quat_wxyz = rotm2quat(R_c2w);
    res.eulZYX_deg = rad2deg(rotm2eul(R_c2w,'ZYX'));
    res.reprojPerCorner = reprojErrs_tag;
    res.meanReproj = meanReproj_tag;
    results = [results; res]; %#ok<AGROW>
end

% show fused reprojection summary
fprintf('Combined-solve using %s. Total points=%d. Overall mean reproj = %.3f px\n', usedMethod, size(allImgPts,1), mean(reprojErrAll));

figure;
imshow(I); hold off;

showExtrinsics(results, cameraParams);

%% ---------------- helper: load_cameraParams ----------------
function cameraParams = load_cameraParams()
    candidates = { fullfile(pwd,'cameraParams.mat'), fullfile(pwd,'..','..','cache','camera','cameraParams.mat'), fullfile('..','cache','camera','cameraParams.mat') };
    for i=1:numel(candidates)
        f = candidates{i};
        if exist(f,'file')
            s = load(f);
            fn = fieldnames(s);
            cameraParams = s.(fn{1});
            return;
        end
    end
    error('cameraParams.mat not found. Save cameraParams (from estimateCameraParameters) to current folder or ../../cache/camera/ .');
end

%% ---------------- helper: load_tagWorldMap ----------------
function tagWorldMap = load_tagWorldMap()
    candidates = { fullfile(pwd,'tagWorldMap.mat'), fullfile(pwd,'..','..','cache','camera','tagWorldMap.mat'), fullfile('..','cache','camera','tagWorldMap.mat') };
    for i=1:numel(candidates)
        f = candidates{i};
        if exist(f,'file')
            s = load(f);
            if isfield(s,'tagWorldMap')
                tagWorldMap = s.tagWorldMap;
            else
                fn = fieldnames(s); tagWorldMap = s.(fn{1});
            end
            return;
        end
    end
    error('tagWorldMap.mat not found. Generate it with the provided generator and place it in current folder or ../../cache/camera/ .');
end

%% ---------------- visualization: showExtrinsics ----------------
function showExtrinsics(results, cameraParams)
    if nargin < 2
        cameraParams = load_cameraParams();
    end
    if isempty(results)
        error('results empty');
    end
    figure('Name','Per-tag Extrinsics (estworldpose)','Color','w'); hold on; grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    for i=1:numel(results)
        wc = results(i).worldCorners; % 4x3
        patch('XData', wc(:,1), 'YData', wc(:,2), 'ZData', wc(:,3), 'FaceColor', [0.9 0.9 0.9], 'FaceAlpha', 0.6);
        plot3([wc(:,1); wc(1,1)], [wc(:,2); wc(1,2)], [wc(:,3); wc(1,3)], '-k', 'LineWidth', 1);
        cen = mean(wc,1);
        text(cen(1), cen(2), cen(3)+0.02, sprintf('id=%d', results(i).id), 'FontSize', 9, 'Color', 'k');
    end
    % draw cameras
    for i=1:numel(results)
        T = results(i).T_world_from_cam;
        drawCameraFrustum(T, cameraParams, 0.3, 0.1);
        pos = results(i).cam_position_world;
        text(pos(1), pos(2), pos(3)+0.05, sprintf('id=%d e=%.2fpx', results(i).id, results(i).meanReproj), 'Color','b');
    end
    view(3); camorbit(20,-10);
    hold off;
    title('Per-tag camera poses (world frame) - one pose per tag');
end

%% ---------------- draw frustum helper ----------------
function drawCameraFrustum(T_world_from_cam, cameraParams, depth, axisScale)
    if nargin<3, depth = 0.5; end
    if nargin<4, axisScale = 0.1; end
    if isprop(cameraParams,'Intrinsics')
        intr = cameraParams.Intrinsics;
    else
        intr = cameraParams;
    end
    imgSize = intr.ImageSize; W = imgSize(2); H = imgSize(1);
    fx = intr.FocalLength(1); fy = intr.FocalLength(2);
    cx = intr.PrincipalPoint(1); cy = intr.PrincipalPoint(2);
    imgPts = [1 1; W 1; W H; 1 H]; % TL TR BR BL
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
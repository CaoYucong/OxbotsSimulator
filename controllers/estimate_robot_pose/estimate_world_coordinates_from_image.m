function results = estimate_world_coordinates_from_image(intrStruct, temp_img_path)
% ESTIMATE_WORLD_COORDINATES_FROM_IMAGE
%   results = estimate_world_coordinates_from_image(intrStruct, temp_img_path)
%
% Inputs:
%   intrStruct    - camera intrinsics input. Supported forms:
%                   * cameraIntrinsics object
%                   * struct with fields 'FocalLength','PrincipalPoint','ImageSize'
%                   * struct with field 'intr' that is cameraIntrinsics
%   temp_img_path - (optional) path to input image. Default: '../../cache/camera/temp_img.png'
%
% Output:
%   results       - struct array, same as your original script's results
%
% NOTE: other data (tagWorldMap) are loaded using the same helper logic as before.

if nargin < 2 || isempty(temp_img_path)
    temp_img_path = "../../cache/camera/temp_img.png";
end

% ---- create intr (cameraIntrinsics) and cameraParams (cameraParameters) from input ----
[intr, cameraParams] = build_intrinsics_from_input(intrStruct);

% ---- Read the input image ----
I = imread(temp_img_path);

% ---- AprilTag detection ----
tagFamily = "tag36h11";
[ids, loc, detectedFamily] = readAprilTag(I, tagFamily);
results = -1; % default empty output
if isempty(ids)
    warning('No AprilTags detected in %s', temp_img_path);
    return;
end
if size(loc, 3) == 1
    warning('Only 1 AprilTag detected in %s, skip', temp_img_path);
    return;
end

% ---- Draw markers on detected tag corners ----
for idx = 1:length(ids)
    disp("Detected Tag ID, Family: " + ids(idx) + ", " + detectedFamily(idx));
    markerRadius = 2;
    numCorners = size(loc, 1);
    markerPosition = [loc(:, :, idx), repmat(markerRadius, numCorners, 1)];
    I = insertShape(I, "Circle", [loc(1, :, idx), 5], ShapeColor="red", Opacity=1);
end

% ---- Build output path: same folder, fixed filename ----
[folder, ~, ~] = fileparts(temp_img_path);
if isempty(folder)
    folder = pwd;
end
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

% --- load tagWorldMap (unchanged behavior) ---
tagWorldMap = load_tagWorldMap();

% build id->worldCorners map
mapById = containers.Map('KeyType','double','ValueType','any');
for k = 1:numel(tagWorldMap)
    mapById(double(tagWorldMap(k).id)) = tagWorldMap(k).worldCorners; % 4x3
end

% check availability of estimateWorldCameraPose and estworldpose
hasEstWorld = exist('estimateWorldCameraPose','file') == 2;
hasEstWorldPoseNew = exist('estworldpose','file') == 2;

results = struct([]);
for k = 1:N
    id = ids(k);
    corners_img = squeeze(loc3(:,:,k)); % 4x2
    if any(isnan(corners_img(:)))
        warning('Tag %d: some corners NaN, skipping', id);
        continue;
    end
    if ~isKey(mapById, id)
        warning('Tag %d not in tagWorldMap, skipping', id);
        continue;
    end
    worldCorners = mapById(id); % 4x3, BL,BR,TR,TL
    
    % undistort points (preferred) - use constructed cameraParams
    try
        ptsUnd = undistortPoints(corners_img, cameraParams); % 4x2
    catch
        ptsUnd = corners_img;
    end

    % annotate world coords near detected corners (same as before)
    I = insertText(I, ptsUnd(:,:), "(" + worldCorners(:,1) + "," + worldCorners(:,2) + ...
        "," + worldCorners(:,3) + ")", 'FontSize', 20, 'BoxColor', 'black', 'TextColor', 'white');
    
    % try estworldpose for single-tag case (preferred)
    usedMethod = '';
    orientation = []; location = [];
    if isempty(orientation)
        try
            [worldPose, inlierIdx, status] = estworldpose(ptsUnd, worldCorners, intr, ...
                MaxReprojectionError=8, Confidence=95, MaxNumTrials=200);
            if status == 0
                R_c2w = worldPose.R;            % camera -> world
                cam_pos = worldPose.Translation; % 1x3
                orientation = R_c2w';   % world -> cam
                location = cam_pos;     % camera location in world coords
                usedMethod = 'estworldpose';
            else
                warning('estworldpose status=%d for id=%d. Falling back to extrinsics.', status, id);
            end
        catch ME
            warning('estworldpose failed. Falling back to extrinsics for id=%d.', id);
        end
    end
    
    % if not solved by estworldpose (or N>1), try estimateWorldCameraPose (original behavior) or extrinsics fallback
    if isempty(orientation)
        if hasEstWorld
            try
                [orientation, location, inlierIdx] = estimateWorldCameraPose(ptsUnd, worldCorners, cameraParams, ...
                    'MaxReprojectionError', 4, 'Confidence', 99);
                usedMethod = 'estimateWorldCameraPose';
            catch ME
                warning('estimateWorldCameraPose failed for id=%d: %s. Falling back to extrinsics.', id, ME.message);
            end
        end
    end
    
    % fallback to extrinsics if necessary (extrinsics requires cameraParameters)
    if isempty(orientation)
        try
            [R_ex, t_ex] = extrinsics(ptsUnd, worldCorners, cameraParams);
            orientation = R_ex;
            location = (-orientation' * t_ex')';
            usedMethod = 'extrinsics';
        catch ME
            warning('extrinsics failed for id=%d: %s. Skipping.', id, ME.message);
            continue;
        end
    end
    
    % Build T_world_from_cam
    R_c2w = orientation';
    cam_pos = location(:)'; % ensure row
    T_world_from_cam = eye(4);
    T_world_from_cam(1:3,1:3) = R_c2w;
    T_world_from_cam(1:3,4) = cam_pos';
    
    % For reprojection: compute t_ex = -R * C (R = world->cam)
    t_ex = (-(orientation * cam_pos'))'; % 1x3
    
    % reprojection check (project worldCorners -> image pixels)
    tform = rigidtform3d(R_c2w', -cam_pos);                                % cam_pos 为 1x3（相机在 world 的位置）
    proj = world2img(worldCorners, tform, intr, 'ApplyDistortion', true);   % 返回 Mx2 像素点
    
    reprojErrs = sqrt(sum((proj - corners_img).^2, 2));
    meanReproj = mean(reprojErrs);
    
    % convert orientation->euler/quaternion for readability (camera-to-world rotation)
    quat = rotm2quat(R_c2w); % [w x y z]
    eulZYX = rotm2eul(R_c2w, 'ZYX'); % rad
    
    % pack result
    res.id = id;
    res.corners_img = corners_img;
    res.corners_undistorted = ptsUnd;
    res.worldCorners = worldCorners;
    res.method = usedMethod;
    res.orientation = orientation; % world->cam
    res.location = cam_pos;        % camera position in world
    res.T_world_from_cam = T_world_from_cam;
    res.cam_position_world = cam_pos;
    res.quat_wxyz = quat;
    res.eulZYX_deg = rad2deg(eulZYX);
    res.reprojPerCorner = reprojErrs;
    res.meanReproj = meanReproj;
    
    results = [results; res]; %#ok<AGROW>
end

% show image (annotated)
figure; imshow(I); hold off;

% print brief summary
fprintf('Per-tag estimation done. %d results produced.\n', numel(results));
for i=1:numel(results)
    fprintf(' id=%2d  method=%s  meanReproj=%.3f px  camPos=[%.3f %.3f %.3f]\n', ...
        results(i).id, results(i).method, results(i).meanReproj, results(i).cam_position_world);
end
fprintf('Use showExtrinsics(results, cameraParams) to visualize.\n');

% keep compatibility: return results and optionally display extrinsics
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

%% ---------------- helper: load_tagWorldMap (unchanged) ----------------
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

%% ---------------- visualization: showExtrinsics (unchanged) ----------------
function showExtrinsics(results, cameraParams)
    if nargin < 2
        candidates = { fullfile(pwd,'theoretical_intrinsics_webots.mat'), fullfile(pwd,'..','..','cache','camera','theoretical_intrinsics_webots.mat') };
        if exist(candidates{1},'file')
            s = load(candidates{1}); intr = s.intr;
            camMat = intr.IntrinsicMatrix; imageSize = intr.ImageSize;
            cameraParams = cameraParameters('IntrinsicMatrix', camMat, 'ImageSize', imageSize, 'RadialDistortion',[0 0], 'TangentialDistortion',[0 0]);
        else
            error('No cameraParams provided and theoretical_intrinsics_webots.mat not found.');
        end
    end
    if isempty(results)
        error('results empty');
    end
    figure('Name','Per-tag Extrinsics (estimateWorldCameraPose)','Color','w'); hold on; grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    for i=1:numel(results)
        wc = results(i).worldCorners; % 4x3
        patch('XData', wc(:,1), 'YData', wc(:,2), 'ZData', wc(:,3), 'FaceColor', [0.9 0.9 0.9], 'FaceAlpha', 0.6);
        plot3([wc(:,1); wc(1,1)], [wc(:,2); wc(1,2)], [wc(:,3); wc(1,3)], '-k', 'LineWidth', 1);
        cen = mean(wc,1);
        text(cen(1), cen(2), cen(3), sprintf('id=%d', results(i).id), 'FontSize', 9, 'Color', 'k');
    end
    % draw cameras
    for i=1:numel(results)
        T = results(i).T_world_from_cam;
        drawCameraFrustum(T, cameraParams, 0.3, 0.1);
        pos = results(i).cam_position_world;
        text(pos(1), pos(2), pos(3), sprintf('id=%d e=%.2fpx', results(i).id, results(i).meanReproj), 'Color','b');
    end
    view(3); camorbit(20,-10);
    hold off;
    title('Per-tag camera poses (world frame) - one pose per tag');
end

%% ---------------- draw frustum helper (unchanged) ----------------
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
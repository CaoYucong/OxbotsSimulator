% checkAprilTagCounts_moveBad.m
% Scan images in the 'aprilTagCalibImages' folder adjacent to this script,
% detect AprilTags, save results to CSV, and move any image with fewer than
% minTagsRequired detected tags (or with an error) into a subfolder named
% 'insufficient_tags' within the same folder.
%
% Requirement:
%   - MATLAB Computer Vision Toolbox (readAprilTag)

clear; close all; clc;

% ========== Configuration ==========
% Build imgFolder relative to the script location so the script is robust
scriptDir = fileparts(mfilename('fullpath'));   % directory of this .m file
imgFolder = fullfile(scriptDir, 'aprilTagCalibImages'); % image directory to scan

tagFamily = 'tag36h11';               % AprilTag family (modify if needed)
minTagsRequired = 40;                 % Minimum required number of tags
outputCSV = fullfile(scriptDir, 'apriltag_detection_summary.csv');
badSubfolderName = 'insufficient_tags';  % name of subfolder to move bad images into
% ===================================

% Supported image extensions
exts = {'*.png','*.jpg','*.jpeg','*.tif','*.bmp','*.gif','*.JPG','*.PNG','*.JPEG','*.TIF','*.BMP','*.GIF'};

% Collect all image files (non-recursive)
fileList = [];
for k = 1:numel(exts)
    files = dir(fullfile(imgFolder, exts{k}));
    fileList = [fileList; files]; %#ok<AGROW>
end

if isempty(fileList)
    error('No images found in %s. Please check the path.', imgFolder);
end

% Prepare result containers
nFiles = numel(fileList);
fileNames = strings(nFiles,1);
detectedCounts = zeros(nFiles,1);
notes = strings(nFiles,1);

fprintf('Scanning %d images for AprilTags (family = %s)...\n', nFiles, tagFamily);

% Create bad-images subfolder path (do not create yet; create when needed)
badFolder = fullfile(imgFolder, badSubfolderName);

for i = 1:nFiles
    fname_full = fullfile(fileList(i).folder, fileList(i).name);
    fileNames(i) = string(fileList(i).name);

    try
        % Read image (color or grayscale both supported)
        I = imread(fname_full);

        % Detect AprilTags
        [tagIds, tagLocs] = readAprilTag(I, tagFamily); %#ok<ASGLU>

        % Count detected tags
        if isempty(tagIds)
            count = 0;
        else
            count = numel(tagIds);
        end
        detectedCounts(i) = count;

        % Report and move if below threshold
        if count < minTagsRequired
            notes(i) = sprintf('Less than %d tags: %d', minTagsRequired, count);
            fprintf('File: %s  --> detected: %d (BELOW threshold)  --> will move\n', ...
                fileList(i).name, count);

            % create bad folder if needed
            if ~exist(badFolder, 'dir')
                mkdir(badFolder);
            end

            % move the file safely (avoid overwriting existing file in badFolder)
            destName = uniqueDestination(fullfile(badFolder, fileList(i).name));
            movefile(fname_full, destName);
        else
            notes(i) = "";
            fprintf('File: %s  --> detected: %d\n', fileList(i).name, count);
        end

    catch ME
        % Catch errors (e.g., corrupted image, unsupported tag family)
        detectedCounts(i) = -1;
        notes(i) = sprintf('Error: %s', ME.message);
        fprintf('File: %s  --> ERROR: %s  --> will move\n', fileList(i).name, ME.message);

        % create bad folder if needed and move
        if ~exist(badFolder, 'dir')
            mkdir(badFolder);
        end
        destName = uniqueDestination(fullfile(badFolder, fileList(i).name));
        try
            movefile(fname_full, destName);
        catch moveErr
            % If move fails, record it in notes
            notes(i) = notes(i) + " | MoveError: " + moveErr.message;
            fprintf('  Move failed for %s : %s\n', fileList(i).name, moveErr.message);
        end
    end
end

% Save results to CSV
T = table(fileNames, detectedCounts, notes, ...
    'VariableNames', {'FileName','DetectedCount','Notes'});
writetable(T, outputCSV);

fprintf('Detection finished. Results saved to %s\n', outputCSV);

% List images that failed or are below threshold
badIdx = find((detectedCounts < minTagsRequired) | (detectedCounts < 0));
if isempty(badIdx)
    fprintf('All images have detected at least %d tags.\n', minTagsRequired);
else
    fprintf('\nImages with insufficient detections or errors (moved to %s):\n', badFolder);
    for j = 1:numel(badIdx)
        idx = badIdx(j);
        fprintf(' - %s : %s\n', fileNames(idx), notes(idx));
    end
end

%% Helper: uniqueDestination
% Ensures the destination filename does not overwrite an existing file.
% If dest exists, appends suffix "_1", "_2", ... before file extension.
function dest = uniqueDestination(destPath)
    [p, n, e] = fileparts(destPath);
    dest = destPath;
    counter = 1;
    while exist(dest, 'file')
        dest = fullfile(p, sprintf('%s_%d%s', n, counter, e));
        counter = counter + 1;
    end
end
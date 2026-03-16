%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Project: UniBots 2026 – Vision Calibration Pipeline
% Script:  AprilTag-Based Camera Calibration (MATLAB)
%
% Description:
% This script performs full camera calibration using AprilTag markers, 
% following the workflow recommended for UniBots 2026 robot vision. 
% The procedure:
%   1. Loads AprilTag images and generates a synthetic calibration pattern.
%   2. Detects AprilTag IDs and corner locations.
%   3. Converts AprilTag corners into checkerboard-equivalent points.
%   4. Detects calibration points from captured robot camera images.
%   5. Generates world coordinates and estimates camera intrinsics/extrinsics.
%   6. Visualizes reprojection errors and camera pose.
%   7. Produces undistorted output images for verification.
%
% This calibration pipeline allows all camera observations to be mapped into 
% a normalized camera coordinate frame, enabling robust localization, 
% AprilTag detection, and line-following perception in the UniBots 2026 arena.
%
% Notes:
% - The script assumes AprilTag family "tag36h11", consistent with the 
%   UniBots competition arena markers.
% - World coordinates follow checkerboard-style layout derived from tags.
% - No modifications have been made to the user's original code logic.
%
% Author: (Your Name)
% Date:   (Update as needed)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

downloadURL  = "https://github.com/AprilRobotics/apriltag-imgs/archive/master.zip";
dataFolder   = fullfile(tempdir,"apriltag-imgs",filesep); 
options      = weboptions('Timeout', Inf);
zipFileName  = fullfile(dataFolder,"apriltag-imgs-master.zip");
folderExists = exist(dataFolder,"dir");

% Create a folder in a temporary directory to save the downloaded file.
if ~folderExists  
    mkdir(dataFolder); 
    disp("Downloading apriltag-imgs-master.zip (60.1 MB)...") 
    websave(zipFileName,downloadURL,options); 
    
    % Extract contents of the downloaded file.
    disp("Extracting apriltag-imgs-master.zip...") 
    unzip(zipFileName,dataFolder); 
end

% Define the ZIP file path for the downloaded AprilTag dataset.
zipFileName  = fullfile(dataFolder,"apriltag-imgs-master.zip");

% Check whether the target data folder already exists.
folderExists = exist(dataFolder,"dir");

% -------------------------------------------------------------------------
% Set the properties of the calibration pattern.
% -------------------------------------------------------------------------

% Specify the arrangement of AprilTags in the calibration pattern.
tagArrangement = [5,8];

% Specify the AprilTag family used for calibration.
tagFamily = "tag36h11";

% Generate the AprilTag calibration pattern.
tagImageFolder = fullfile(dataFolder,"apriltag-imgs-master",tagFamily);

% Create an imageDatastore to manage tag image files.
imdsTags = imageDatastore(tagImageFolder);

% Construct the printed calibration pattern using the tag images.
calibPattern = helperGenerateAprilTagPattern(imdsTags,tagArrangement,tagFamily);

% -------------------------------------------------------------------------
% Detect and process AprilTag locations in the calibration pattern.
% -------------------------------------------------------------------------

% Detect AprilTags and extract their IDs and corner locations.
[tagIds, tagLocs] = readAprilTag(calibPattern,tagFamily);

% Sort the detected tags by their ID values.
[~, sortIdx] = sort(tagIds);
tagLocs = tagLocs(:,:,sortIdx);

% Reshape corner locations to an M-by-2 matrix.
tagLocs = reshape(permute(tagLocs,[1,3,2]),[],2);

% Convert AprilTag corner layout to checkerboard-style indexing.
checkerIdx = helperAprilTagToCheckerLocations(tagArrangement);

% Select corresponding checkerboard corner points.
imagePoints = tagLocs(checkerIdx(:),:);

% Display detected corner locations over the calibration pattern image.
figure; imshow(calibPattern); hold on
plot(imagePoints(:,1),imagePoints(:,2),"ro-",MarkerSize=15)

% -------------------------------------------------------------------------
% Detect the calibration pattern from real captured images.
% -------------------------------------------------------------------------

% Create an imageDatastore for the calibration photos.
imdsCalib = imageDatastore("aprilTagCalibImages/");

% Detect AprilTag-based checkerboard corners in calibration images.
[imagePoints,boardSize] = helperDetectAprilTagCorners(imdsCalib,tagArrangement,tagFamily);

% -------------------------------------------------------------------------
% Generate world coordinates and estimate camera parameters.
% -------------------------------------------------------------------------

% Define the physical tag size in millimeters.
tagSize = 20; % mm

% Generate world coordinates for the checkerboard points.
worldPoints = patternWorldPoints("checkerboard",boardSize, tagSize);

% Read the first calibration image to determine image size.
I = readimage(imdsCalib,1);
imageSize = size(I,1:2);

% Estimate intrinsic and extrinsic camera parameters.
params = estimateCameraParameters(imagePoints,worldPoints,ImageSize=imageSize);

% -------------------------------------------------------------------------
% Visualize calibration quality.
% -------------------------------------------------------------------------

% Display reprojection errors for all calibration images.
figure
showReprojectionErrors(params)

% Display camera extrinsic parameters.
figure
showExtrinsics(params)

% -------------------------------------------------------------------------
% Undistort and display example images.
% -------------------------------------------------------------------------

% Read a sample calibration image.
I = readimage(imdsCalib,9);

% Insert markers for the detected and reprojected points.
J3 = I;
J3 = insertMarker(J3,imagePoints(:,:,9),"o",MarkerColor="g",Size=5);
J3 = insertMarker(J3,params.ReprojectedPoints(:,:,9),"x",MarkerColor="r",Size=5);

% Display the image.
figure
imshow(J3)

% Undistort using default output view.
J1 = undistortImage(I,params);
figure; imshowpair(I,J1,'montage');
title('Original Image (left) vs. Corrected Image (right)');

% Undistort using full output view.
J2 = undistortImage(I,params,'OutputView','full');
figure; 
imshow(J2);
title('Full Output View');

save('../../controllers/epuck_camera_reader/cameraParams.mat','params','-v7.3');
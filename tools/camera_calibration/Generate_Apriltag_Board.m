% Set the properties of the calibration pattern.
tagArrangement = [5,8];
tagFamily = "tag36h11";

% Generate the calibration pattern using AprilTags.
tagImageFolder = fullfile(dataFolder,"apriltag-imgs-master",tagFamily);
imdsTags = imageDatastore(tagImageFolder);
calibPattern = helperGenerateAprilTagPattern(imdsTags,tagArrangement,tagFamily);
imwrite(calibPattern, 'aprilTagBoard.png');

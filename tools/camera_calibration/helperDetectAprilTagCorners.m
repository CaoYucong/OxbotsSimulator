function [imagePoints,boardSize,imagesUsed] = helperDetectAprilTagCorners(imdsCalib,tagArrangement,tagFamily)

% Get the pattern size from tagArrangement.
boardSize = tagArrangement*2 + 1;

% Initialize number of images and tags.
numImages = length(imdsCalib.Files);
numTags = tagArrangement(1)*tagArrangement(2);

% Initialize number of corners in AprilTag pattern.
imagePoints = zeros(numTags*4,2,numImages);
imagesUsed = zeros(1,numImages);

% Get checkerboard corner indices from AprilTag corners.
checkerIdx = helperAprilTagToCheckerLocations(tagArrangement);

for idx = 1:numImages

    % Read and detect AprilTags in image.
    I = readimage(imdsCalib,idx);
    [tagIds,tagLocs] = readAprilTag(I,tagFamily);

    % figure;imshow(I); hold on
    % % ---- Draw markers on detected tag corners ----
    % for drawing_idx = 1:length(tagIds)
    %     plot(tagLocs(:,1,drawing_idx),tagLocs(:,2,drawing_idx),"ro-",MarkerSize=15)
    % end

    % Accept images if all tags are detected.
    if numel(tagIds) == numTags
        % Sort detected tags using ID values.
        [~,sortIdx] = sort(tagIds);
        tagLocs = tagLocs(:,:,sortIdx);
        
        % Reshape tag corner locations into a M-by-2 array.
        tagLocs = reshape(permute(tagLocs,[1,3,2]),[],2);
        
        % Populate imagePoints using checkerboard corner indices.
        imagePoints(:,:,idx) = tagLocs(checkerIdx(:),:);
        imagesUsed(idx) = true; 
    else
        imagePoints(:,:,idx) = zeros(160,2);
    end
    
end

end
function output_path = recognise_temp_image(temp_img_path)
% recognise_temp_img  
% Perform AprilTag detection on the input image, draw markers,
% and save the processed image as temp_img_recognised.png in the same folder.
%
% Input:
%   temp_img_path  - string or char, e.g. "../../cache/camera/temp_img.png"
%
% Output:
%   output_path    - full path of output image (char)

    % ---- Read the input image ----
    I = imread(temp_img_path);

    % ---- AprilTag detection ----
    tagFamily = "tag36h11";
    [id, loc, detectedFamily] = readAprilTag(I, tagFamily);

    % ---- Draw markers on detected tag corners ----
    for idx = 1:length(id)
        disp("Detected Tag ID, Family: " + id(idx) + ", " + detectedFamily(idx));

        markerRadius = 2;
        numCorners = size(loc, 1);
        markerPosition = [loc(:, :, idx), repmat(markerRadius, numCorners, 1)];

        I = insertShape(I, "FilledCircle", markerPosition, ...
                        ShapeColor="red", Opacity=1);
    end

    % ---- Build output path: same folder, fixed filename ----
    [folder, ~, ~] = fileparts(temp_img_path);
    output_path = fullfile(folder, "temp_img_recognised.png");

    % ---- Save the processed image ----
    imwrite(I, output_path);

    % ---- Display final result ----
    imshow(I);
end
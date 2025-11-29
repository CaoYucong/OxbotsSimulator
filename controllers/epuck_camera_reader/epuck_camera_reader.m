function epuck_camera_reader_fixed
TIME_STEP = 64;
sample_every_frame = 100;

camera = wb_robot_get_device('camera');
wb_camera_enable(camera, TIME_STEP);

width  = wb_camera_get_width(camera);
height = wb_camera_get_height(camera);

MAX_SPEED = 6.28;

% get a handler to the motors and set target position to infinity (speed control)
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);

% set up the motor speeds at 10% of the MAX_SPEED.
wb_motor_set_velocity(left_motor, 0.5 * MAX_SPEED);
wb_motor_set_velocity(right_motor, 0.3 * MAX_SPEED);

loop_counter = 0;
image_file_num = 1;
while wb_robot_step(TIME_STEP) ~= -1
    raw = wb_camera_get_image(camera);

    % if mod(loop_counter, sample_every_frame) == 0
    %   filename = sprintf('../../cache/camera/image_%04d.png', image_file_num); % 例如 "image_0001.png"
    %   image_file_num = image_file_num + 1;
    %   success = wb_camera_save_image(camera, filename, 100);
    % end
    success = wb_camera_save_image(camera, '../../cache/camera/temp_img.png', 100);

    % Apriltag Recognition
    recognise_temp_image("../../cache/camera/temp_img.png");

    % delete('../../cache/camera/temp_img.png');
end

end
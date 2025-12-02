% ====== Webots camera intrinsics (theoretical) =======
fov = 1.57;      % horizontal field of view (radians)
W = 1280;        
H = 960;

fx = (W/2) / tan(fov/2);
fy = fx;
cx = W/2;
cy = H/2;

intr = cameraIntrinsics([fx fy], [cx cy], [H W]);

% ====== Save into your folder =======
save('../../controllers/epuck_camera_reader/theoretical_intrinsics_webots.mat', 'intr');  % 改成你想放的位置

fprintf('Saved intrinsics to ../../controllers/epuck_camera_reader/theoretical_intrinsics_webots.mat\n');
disp(intr)
function y = stereo_projection(Pose, landmark)

    % calculate local<dx, dy, dz>
    R = Pose(1:3,1:3);
    P = Pose(1:3,4);

    Local = R'*(landmark-P);

    % calibration parameters, typical Kinect
    fx = 525.0;
    fy = 525.0;
    cx0 = 639.5;
    cy0 = 479.5;

    
    % Feature co-ords in camera frame
    p3d1 = Local;
    ul = fx * p3d1(1) / p3d1(3) + cx0;
    v = fy * p3d1(2) / p3d1(3) + cy0;
    ur = 0;
    oriy = [ul ur v Local'];
    
    % filter the measurements which are in the correct range
    row_idx = ( oriy(:, 1) < cx0*2+1 && oriy(:, 1) > 0.0 && ...
                oriy(:, 3) < cy0*2+1 && oriy(:, 3) > 0.0);
    y = oriy(row_idx, :);
end
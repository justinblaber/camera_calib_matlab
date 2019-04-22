function test_undistort_array
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load distorted data
    load(fullfile(tests_path, 'data', 'checker', '1_d.mat'));

    % Undistort
    undistort_array_interp = 'spline';
    obj_distortion = class.distortion.base(distortion.heikkila97, []);
    A = [578.8932,        0, 331.1893;
         0,        578.8932, 244.5372;
         0,               0,        1];
    d = [0.1738; 0.1930; 0.0025; 0.0080];
    array = alg.undistort_array(checker1_d, obj_distortion, A, d, undistort_array_interp);

    % Assert
    abs_diff = abs(array - rgb2gray(im2double(imread(fullfile(tests_path, 'data', 'checker', '1.jpg')))));
    abs_diff(isnan(abs_diff)) = 0;
    assert(all(all(abs_diff < 0.075)));
end

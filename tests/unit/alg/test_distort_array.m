function test_distort_array
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load image data
    array = rgb2gray(im2double(imread(fullfile(tests_path, 'data', 'checker', '1.jpg'))));

    % Distort
    opts.p_p_d2p_p_it_cutoff = 20;
    opts.p_p_d2p_p_norm_cutoff = '1e-6';
    distort_array_interp = 'spline';
    obj_distortion = class.distortion.base(distortion.heikkila97, opts);
    A = [578.8932,        0, 331.1893;
         0,        578.8932, 244.5372;
         0,               0,        1];
    d = [0.1738; 0.1930; 0.0025; 0.0080];
    array_d = alg.distort_array(array, obj_distortion, A, d, distort_array_interp);

    % Assert
    load(fullfile(tests_path, 'data', 'checker', '1_d.mat'));
    assert(all(all(abs(checker1_d - array_d) < 1e-4)));
end

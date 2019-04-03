function test_refine_checker_point
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'refine_checker_point.mat'));

    opts.target_optimization = 'edges';
    opts.refine_checker_min_hw = 4;
    opts.refine_checker_max_hw = 15;
    opts.refine_checker_opencv_it_cutoff = 20;
    opts.refine_checker_opencv_norm_cutoff = 1e-3;
    opts.refine_checker_edges_h2_init = 0.75;
    opts.refine_checker_edges_it_cutoff = 20;
    opts.refine_checker_edges_norm_cutoff = 1e-6;
    opts.dominant_grad_angles_num_bins = 20;
    opts.dominant_grad_angles_space_peaks = 1;
    opts.verbosity = 0;
    [p_cb_p, cov_cb_p] = alg.refine_checker_point(p_cb_p_init, ...
                                                  boundary_p_center, ...
                                                  array_cb, ...
                                                  array_cb_dx, ...
                                                  array_cb_dy, ...
                                                  opts);

    %{
    % Plot example
    f = figure;
    imshow(array_cb, []);
    hold on;
    plot(boundary_p_center([1 2 3 4 1], 1) + p_cb_p_init(1), boundary_p_center([1 2 3 4 1], 2) + p_cb_p_init(2), '-r');
    plot(p_cb_p_init(1), p_cb_p_init(2), 'rs');
    plot(p_cb_p(1), p_cb_p(2), 'gs');
    close(f);
    %}

    % Assert
    assert(all(all(abs(p_cb_p - 1.0e+02 * [3.537435697370190   2.944115103340532]) < 1e-4)));
    assert(all(all(abs(cov_cb_p - 1.0e-03 * [0.353238877459771   0.001082736011778
                                             0.001082736011778   0.332600351262948]) < 1e-4)));

 	clear

    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'refine_checker_point.mat'));

    opts.target_optimization = 'opencv';
    opts.refine_checker_min_hw = 4;
    opts.refine_checker_max_hw = 15;
    opts.refine_checker_opencv_it_cutoff = 20;
    opts.refine_checker_opencv_norm_cutoff = 1e-3;
    opts.verbosity = 0;
    [p_cb_p, cov_cb_p] = alg.refine_checker_point(p_cb_p_init, ...
                                                  boundary_p_center, ...
                                                  array_cb, ...
                                                  array_cb_dx, ...
                                                  array_cb_dy, ...
                                                  opts);

    %{
    % Plot example
    f = figure;
    imshow(array_cb, []);
    hold on;
    plot(boundary_p_center([1 2 3 4 1], 1) + p_cb_p_init(1), boundary_p_center([1 2 3 4 1], 2) + p_cb_p_init(2), '-r');
    plot(p_cb_p_init(1), p_cb_p_init(2), 'rs');
    plot(p_cb_p(1), p_cb_p(2), 'gs');
    close(f);
    %}

    % Assert
    assert(all(all(abs(p_cb_p - 1.0e+02 * [3.537408536492807   2.943930834735747]) < 1e-4)));
    assert(all(all(abs(cov_cb_p - [0.012269068621633   0.001925870026662;
                                   0.001925870026662   0.016480525165822]) < 1e-4)));
end

function test_refine_ellipse_edges
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'ellipse.mat'));

    opts.refine_ellipse_edges_h2_init = 0.75;
    opts.refine_ellipse_edges_it_cutoff = 20;
    opts.refine_ellipse_edges_norm_cutoff = 1e-6;
    e = alg.refine_ellipse_edges(array_dx, array_dy, e_init, opts);

    %{
    % Plot example
    f = figure;
    imshow(array, []);
    hold on;
    external.ellipse(e(3), e(4), e(5), e(1), e(2), 'g');
    pause(1);
    close(f);
    %}

    % Assert
    assert(all(all(abs(e - [12.948135949893175
                            12.832869003131211
                             5.822077561012249
                             5.045534542908317
                             2.964473871654429]) < 1e-4)));
end

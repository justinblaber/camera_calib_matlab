function test_refine_ellipse_dot
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'ellipse.mat'));

    opts.refine_ellipse_dot_h2_init = 1;
    opts.refine_ellipse_dot_it_cutoff = 20;
    opts.refine_ellipse_dot_norm_cutoff = 1e-6;
    e = alg.refine_ellipse_dot(array, e_init, opts);

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
    assert(all(all(abs(e - [12.958284439381227
                            12.836025313079070
                             5.824305612038439
                             5.040543336893506
                             2.961260053681906]) < 1e-4)));
end

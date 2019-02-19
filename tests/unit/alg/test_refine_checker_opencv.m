function test_refine_checker_opencv
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'checker.mat'));

    [p, cov_p] = alg.refine_checker_opencv(array_dx, array_dy);

    %{
    % Plot example
    f = figure;
    imshow(array_dx.^2 + array_dy.^2, []);
    hold on;
    plot(p_init(1), p_init(2), 'rs');
    plot(p(1), p(2), 'gs');
    pause(1);
    close(f);
    %}

    % Assert
    assert(all(all(abs(p - [15.537134278596810  16.288478666151310]) < 1e-4)));
    assert(all(all(abs(cov_p - [ 0.002823137521089  -0.000319661476055
                                -0.000319661476055   0.002916900778227]) < 1e-4)));
end

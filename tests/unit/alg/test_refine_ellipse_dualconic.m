function test_refine_ellipse_dualconic
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'ellipse.mat'));

    e = alg.refine_ellipse_dualconic(array_dx, array_dy);

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
    assert(all(all(abs(e - [12.941116151474535
                            12.834138458460759
                             5.901990645761262
                             5.057674901422744
                             2.980935941780454]) < 1e-4)));
end

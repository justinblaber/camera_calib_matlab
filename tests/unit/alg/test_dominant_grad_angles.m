function test_dominant_grad_angles
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));

    % Load conic data
    load(fullfile(tests_path, 'data', 'dominant_grad_angles.mat'));

    opts.dominant_grad_angles_num_bins = 20;
    opts.dominant_grad_angles_space_peaks = 1;
    angles_test = alg.dominant_grad_angles(array_dx, array_dy, 2, opts);

    %{
    % Plot example
    f = figure;
    imshow(array_dx.^2 + array_dy.^2, []);
    hold on;
    p = [13.454815113457174  13.135563241128239];
    l1 = alg.pointslope2line(p, tan(angles_test(1) + pi/2));
    l2 = alg.pointslope2line(p, tan(angles_test(2) + pi/2));
    e2 = ezplot(@(x, y) l1(1).*x + l1(2).*y + l1(3), [1 size(array_dx, 2) 1 size(array_dx, 1)]); %#ok<EZPLT>
    e3 = ezplot(@(x, y) l2(1).*x + l2(2).*y + l2(3), [1 size(array_dx, 2) 1 size(array_dx, 1)]); %#ok<EZPLT>
    set(e2, 'color', 'g');
    set(e3, 'color', 'g');
    pause(1);
    close(f);
    %}

    % Assert
    assert(all(all(abs(angles - angles_test) < 1e-4)));
    assert(all(isnan(alg.dominant_grad_angles(nan, nan, 2, opts))));
    assert(all(isnan(alg.dominant_grad_angles([], [], 2, opts))));
end

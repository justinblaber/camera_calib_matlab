function test_refine_checker_opencv
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    
    % Load conic data
    load(fullfile(tests_path,'data','checker.mat'));
    
    [p, cov_p] = alg.refine_checker_opencv(array_dx,array_dy,p_init);
    
    %{
    % Plot example
    f = figure;
    imshow(array_dx.^2 + array_dy.^2,[]);
    hold on;
    plot(p_init(1),p_init(2),'rs');
    plot(p(1),p(2),'gs');
    pause(1);
    close(f);    
    %}
    
    % Assert
    assert(all(all(abs(p - [15.544070694168779  16.256744451714237]) < eps('single'))));
    assert(all(all(abs(cov_p - [ 0.002287291664887  -0.000223903978982
                                -0.000223903978982   0.002296588242969]) < eps('single'))));
end
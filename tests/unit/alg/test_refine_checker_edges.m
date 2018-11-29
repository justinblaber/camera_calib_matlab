function test_refine_checker_edges
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    
    % Load conic data
    load(fullfile(tests_path,'data','checker.mat'));
    
    opts.refine_checker_edges_h2_init = 0.75;
    opts.refine_checker_edges_it_cutoff = 20;
    opts.refine_checker_edges_norm_cutoff = 1e-6;
    [p, cov_p] = alg.refine_checker_edges(array_dx,array_dy,l1,l2,opts);
    
    %{
    % Plot example
    f = figure;
    imshow(array_dx.^2 + array_dy.^2,[]);
    hold on;
    e2 = ezplot(@(x,y) l1(1).*x + l1(2).*y + l1(3),[1 size(array_dx,2) 1 size(array_dx,1)]); %#ok<EZPLT>
    e3 = ezplot(@(x,y) l2(1).*x + l2(2).*y + l2(3),[1 size(array_dx,2) 1 size(array_dx,1)]); %#ok<EZPLT>
    set(e2,'color','r');
    set(e3,'color','r');
    plot(p(1),p(2),'gs');
    pause(1);
    close(f);    
    %}
    
    % Assert
    assert(all(all(abs(p - [15.603424559791394  16.257538384391811]) < 1e-4)));
    assert(all(all(abs(cov_p - 1.0e-04 * [ 0.730409176408907  -0.036790948485737
                                          -0.036790948485737   0.719644439506550]) < 1e-4)));
end
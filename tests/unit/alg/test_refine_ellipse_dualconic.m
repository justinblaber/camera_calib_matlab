function test_refine_ellipse_dualconic
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    
    % Load conic data
    load(fullfile(tests_path,'data','ellipse.mat'));
    
    e = alg.refine_ellipse_dualconic(array_dx,array_dy);
    
    %{
    % Plot example
    f = figure;
    imshow(array_dx.^2 + array_dy.^2,[]);
    hold on;
    external.ellipse(e(3),e(4),e(5),e(1),e(2),'g');
    pause(1);
    close(f);    
    %}
    
    % Assert
    assert(all(all(abs(e - [20.251035697598265
                            17.606971564906960
                             7.649813708973468
                             6.380724599824798
                             2.133246481371850]) < eps('single'))));
end
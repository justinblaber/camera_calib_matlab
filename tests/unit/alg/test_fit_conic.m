function test_fit_conic
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    
    % Load conic data
    load(fullfile(tests_path,'data','conic.mat'));
    
    Aq = alg.fit_conic(array_dx, array_dy);
    
    %{
    % Plot example
    f = figure;
    A = Aq(1,1);
    B = 2*Aq(1,2);
    C = Aq(2,2);
    D = 2*Aq(1,3);
    E = 2*Aq(2,3);
    F = Aq(3,3);    
    imshow(array_dx.^2 + array_dy.^2,[]);
    hold on;
    e1 = ezplot(@(x,y) A.*x.^2+B.*x.*y+C.*y.^2+D.*x+E.*y+F,[1 size(array_dx,2) 1 size(array_dx,1)]); %#ok<EZPLT>
    set(e1,'color','r');
    pause(1)
    close(f);
    %}
    
    % Assert
    assert(all(all(abs(Aq - [-0.022436553922337  -0.003371389615305   0.584404240928748;
                             -0.003371389615305  -0.019213489495077   0.435893147103909;
                              0.584404240928748   0.435893147103909 -20.698649897847577]) < 1e-4)));
end
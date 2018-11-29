function test_conic2ellipse
    Aq = [-0.022436553922337  -0.003371389615305   0.584404240928748;
          -0.003371389615305  -0.019213489495077   0.435893147103909;
           0.584404240928748   0.435893147103909 -20.698649897847577];
    
    e = alg.conic2ellipse(Aq);
    
    %{
    % Plot example
    f = figure;
    A = Aq(1,1);
    B = 2*Aq(1,2);
    C = Aq(2,2);
    D = 2*Aq(1,3);
    E = 2*Aq(2,3);
    F = Aq(3,3);    
    array = zeros(35,38);
    imshow(array,[]);
    hold on;
    e1 = ezplot(@(x,y) A.*x.^2+B.*x.*y+C.*y.^2+D.*x+E.*y+F,[1 size(array,2) 1 size(array,1)]); %#ok<EZPLT>
    set(e1,'color','r');
    pause(1);
    external.ellipse(e(3),e(4),e(5),e(1),e(2),'g');
    pause(1);
    close(f);
    %}
    
    % Assert
    assert(all(all(abs(e - [23.251025401256015;
                            18.606973065070964;
                             7.649815052476453;
                             6.380726808930452;
                             2.133246917273027]) < 1e-4)));
end
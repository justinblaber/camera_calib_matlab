function test_line_line_intersect
    l1 = [-0.003623782028800;
          -0.000196862044249;
           0.112365384116166];
       
    l2 = 1.0e+03 * [ 0.001000000000000;
                    -0.069586340539404;
                     2.225079029539678];
    
    p = alg.line_line_intersect(l1,l2);
    
    %{
    % Plot example
    f = figure;
    array = zeros(62,57);
    imshow(array,[]);
    hold on;
    e2 = ezplot(@(x,y) l1(1).*x + l1(2).*y + l1(3),[1 size(array,2) 1 size(array,1)]); %#ok<EZPLT>
    e3 = ezplot(@(x,y) l2(1).*x + l2(2).*y + l2(3),[1 size(array,2) 1 size(array,1)]); %#ok<EZPLT>
    set(e2,'color','g');
    set(e3,'color','g');
    plot(p(1),p(2),'rs');
    pause(1);
    close(f);
    %}
    
    % Assert
    assert(all(all(abs(p - [29.247846160999831  32.396111912569111]) < eps('single'))));
end
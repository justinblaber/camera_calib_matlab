function test_line_bb_intersect
    l = [-0.003623782028800;
         -0.000196862044249;
          0.112365384116166];
       
    bb = [ 0.500000000000000   0.500000000000000;
          56.500000000000000  61.500000000000000];

                 
    [p1,p2] = alg.line_bb_intersect(l,bb);
    
    %{
    % Plot example
    f = figure;
    array = zeros(62,57);
    imshow(array,[]);
    hold on;
    e1 = ezplot(@(x,y) l(1).*x + l(2).*y + l(3),[1 size(array,2) 1 size(array,1)]); %#ok<EZPLT>
    set(e1,'color','g');
    plot(p1(1),p1(2),'rs');
    plot(p2(1),p2(2),'rs');
    pause(1);
    close(f);
    %}
    
    % Assert
    assert(all(all(abs(p1 - [30.980603193514437   0.500000000000000]) < 1e-4)));    
    assert(all(all(abs(p2 - [27.666776753692503  61.500000000000000]) < 1e-4)));
end
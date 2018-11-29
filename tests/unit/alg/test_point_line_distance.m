function test_point_line_distance
    p1 = [35 25];
     
    l = [0.850000000000000  -1.000000000000000  18.000000000000000];
    
    p2(1) = (l(2)*(l(2)*p1(1)-l(1)*p1(2)) - l(1)*l(3))/(l(1)^2+l(2)^2);
    p2(2) = (l(1)*(-l(2)*p1(1)+l(1)*p1(2)) - l(2)*l(3))/(l(1)^2+l(2)^2);
    
    %{
    % Plot example
    f = figure;
    array = zeros(62,57);
    imshow(array,[]);
    hold on;
    e1 = ezplot(@(x,y) l(1).*x + l(2).*y + l(3),[1 size(array,2) 1 size(array,1)]); %#ok<EZPLT>
    set(e1,'color','g');
    plot(p1(1),p1(2),'rs');
    plot(p2(1),p2(2),'gs');
    pause(1);
    close(f);
    %}
    
    % Assert
    d = alg.point_line_distance(p1,l);    
    assert(all(all(abs(d - sqrt(sum((p1-p2).^2))) < 1e-4)));
end
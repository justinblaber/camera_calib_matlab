function test_points2line
    p1 = [20 35];
    p2 = [40 52];
     
    l = alg.points2line(p1,p2);
    
%{
    % Plot example
    array = zeros(62,57);
    figure;
    imshow(array,[]);
    hold on;
    e1 = ezplot(@(x,y) l(1).*x + l(2).*y + l(3),[1 size(array,2) 1 size(array,1)]);
    set(e1,'color','g');
    plot(p1(1),p1(2),'rs');
    plot(p2(1),p2(2),'rs');
%}
    
    assert(all(all(abs(l - [0.850000000000000  -1.000000000000000  18.000000000000000]) < eps('single'))));
end
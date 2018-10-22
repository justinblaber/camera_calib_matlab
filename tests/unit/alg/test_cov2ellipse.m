function test_cov2ellipse
    cov = [45.777115082408784  -8.032507080350960
           -8.032507080350960  53.456229864890297];
    p = [23.251025401256015  18.606973065070964];

    e = [23.251025401256015;
         18.606973065070964;
          7.649815052476453;
          6.380726808930452;
          2.133246917273027];       
      
%{
    % Plot example
    s = [35 38];
    [y,x] = ndgrid(1:s(1),1:s(2));    
    array = mvnpdf([x(:) y(:)], ...
                   p, ...
                   cov);
    array = reshape(array,s);
    figure;
    imshow(array,[]);
    hold on;
    external.ellipse(e(3),e(4),e(5),e(1),e(2),'g');
%}
          
    assert(all(all(abs(alg.cov2ellipse(cov,p) - e) < eps('single'))));
end
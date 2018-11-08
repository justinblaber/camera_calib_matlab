function test_A2a
    A = [500 0   100;
         0   500 200;
         0   0   1];
    
    a = [500 100 200]';
       
    % Assert
    assert(isequal(alg.A2a(A),a));
end

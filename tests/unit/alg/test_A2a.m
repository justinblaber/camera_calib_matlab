function test_A2a
    A = [500 0   100;
         0   500 200;
         0   0   1];
    
    a = [500 100 200]';
       
    % Assert
    assert(isequal(alg.A2a(A),a));
    
    % Test in case of NaNs    
    A = [nan 0   nan;
         0   nan nan;
         0   0   1];
     
    % Assert
    assert(all(isnan(alg.A2a(A))));
end

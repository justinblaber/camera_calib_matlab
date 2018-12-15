function test_is_p_in_bb
    bb = [3 5;
          10 12];
    
    % Assert
    assert(alg.is_p_in_bb([5 7],bb));
    assert(~alg.is_p_in_bb([11 7],bb));
end
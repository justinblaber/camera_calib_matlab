function test_is_bb_in_bb
    bb1 = [3 5;
           10 12];
    bb2 = [1  2;
           20 21];
    
    % Assert
    assert(alg.is_bb_in_bb(bb1,bb1));
    assert(alg.is_bb_in_bb(bb1,bb2));
    assert(~alg.is_bb_in_bb(bb2,bb1));
end
function test_is_pos_def
    A = eye(3);
    B = A;
    B(1) = -1;
    
    % Assert
    assert(alg.is_pos_def(A));
    assert(~alg.is_pos_def(B));
end

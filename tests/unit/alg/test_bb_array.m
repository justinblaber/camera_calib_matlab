function test_bb_array
    h = 20;
    w = 15;
    array = zeros(h, w);

    bb = alg.bb_array(array);

    % Assert
    assert(isequal(bb, alg.bb_array(array), [1 1; w h]));
end

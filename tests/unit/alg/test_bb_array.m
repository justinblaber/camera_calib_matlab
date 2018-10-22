function test_bb_array
    h = 20;
    w = 15;
    array = zeros(h,w);
    
    bb = alg.bb_array(array);
    
    assert(isequal(bb,alg.bb_array(array),[0.5 0.5; w-0.5 h-0.5]));
end
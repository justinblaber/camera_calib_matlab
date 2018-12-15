function test_get_sub_array_bb
    array = [0.9047    0.5767    0.4899    0.4711    0.5216;
             0.6099    0.1829    0.1679    0.0596    0.0967;
             0.6177    0.2399    0.9787    0.6820    0.8181;
             0.8594    0.8865    0.7127    0.0424    0.8175;
             0.8055    0.0287    0.5005    0.0714    0.7224];

    bb = [2 1;
          4 3];
      
    sub_array = array(bb(1,2):bb(2,2),bb(1,1):bb(2,1));
    
    % Assert
    assert(isequal(alg.get_sub_array_bb(array,bb), sub_array));
end
function test_ndgrid_bb
    bb = [3 5;
          6 8];

    [y, x] = alg.ndgrid_bb(bb);

    % Assert
    assert(isequal(x, [3 4 5 6;
                      3 4 5 6;
                      3 4 5 6;
                      3 4 5 6]));
    assert(isequal(y, [5 5 5 5;
                      6 6 6 6;
                      7 7 7 7;
                      8 8 8 8]));
end

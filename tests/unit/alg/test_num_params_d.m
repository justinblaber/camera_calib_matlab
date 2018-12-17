function test_num_params_d
    syms sym_test1(x_p, y_p, a, x_o, y_o, d1, d2, d3, d4)
    syms sym_test2(x_p, y_p, a, x_o, y_o, d1, d2, d3, d4, d5, d6, d7)

    sym_test1(x_p, y_p, a, x_o, y_o, d1, d2, d3, d4) = [x_p, y_p];
    sym_test2(x_p, y_p, a, x_o, y_o, d1, d2, d3, d4, d5, d6, d7) = [x_p, y_p];

    f_test1 = matlabFunction(sym_test1);
    f_test2 = matlabFunction(sym_test2);

    % Assert
    assert(alg.num_params_d(sym_test1) == 4)
    assert(alg.num_params_d(sym_test2) == 7)
    assert(alg.num_params_d(f_test1) == 4)
    assert(alg.num_params_d(f_test2) == 7)
end

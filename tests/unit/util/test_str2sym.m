function test_str2sym
    % Set symbolic function string
    str_f = 'sym_f(x_p, y_p, a, x_o, y_o, k1, k2, p1, p2) = [ x_o + a*(p2*((3*(x_o - x_p)^2)/a^2 + (y_o - y_p)^2/a^2) - ((x_o - x_p)*(k1*((x_o - x_p)^2/a^2 + (y_o - y_p)^2/a^2) + k2*((x_o - x_p)^2/a^2 + (y_o - y_p)^2/a^2)^2 + 1))/a + (2*p1*(x_o - x_p)*(y_o - y_p))/a^2), y_o + a*(p1*((x_o - x_p)^2/a^2 + (3*(y_o - y_p)^2)/a^2) - ((y_o - y_p)*(k1*((x_o - x_p)^2/a^2 + (y_o - y_p)^2/a^2) + k2*((x_o - x_p)^2/a^2 + (y_o - y_p)^2/a^2)^2 + 1))/a + (2*p2*(x_o - x_p)*(y_o - y_p))/a^2)]';

    % Set symbolic function
    syms sym_f(x_p, y_p, a, x_o, y_o, k1, k2, p1, p2)
    sym_f(x_p, y_p, a, x_o, y_o, k1, k2, p1, p2) = [ x_o + a*(p2*((3*(x_o - x_p)^2)/a^2 + (y_o - y_p)^2/a^2) - ((x_o - x_p)*(k1*((x_o - x_p)^2/a^2 + (y_o - y_p)^2/a^2) + k2*((x_o - x_p)^2/a^2 + (y_o - y_p)^2/a^2)^2 + 1))/a + (2*p1*(x_o - x_p)*(y_o - y_p))/a^2), y_o + a*(p1*((x_o - x_p)^2/a^2 + (3*(y_o - y_p)^2)/a^2) - ((y_o - y_p)*(k1*((x_o - x_p)^2/a^2 + (y_o - y_p)^2/a^2) + k2*((x_o - x_p)^2/a^2 + (y_o - y_p)^2/a^2)^2 + 1))/a + (2*p2*(x_o - x_p)*(y_o - y_p))/a^2)];

    % Assert
    assert(isequal(util.str2sym(str_f), sym_f));
    assert(strcmp(util.sym2str(util.str2sym(str_f)), str_f));
end

function test_verbose_disp
    out = evalc('util.verbose_disp(''test'', 1, struct(''verbosity'', 1))');
    assert(strcmp(out, ['test' newline]));
end

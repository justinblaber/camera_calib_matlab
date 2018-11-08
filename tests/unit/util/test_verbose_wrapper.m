function test_verbose_wrapper
    out = evalc('util.verbose_wrapper(@fprintf,''test'',1,struct(''verbosity'',1))');
    assert(strcmp(out,'test'));
end

function test_verbose_fprintf
    out = evalc('util.verbose_fprintf(''test'',1,struct(''verbosity'',1))');
    assert(strcmp(out,'test'));
end

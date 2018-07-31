function test_array_interp
    array = [0.7792    0.0119    0.5285    0.6892    0.9133;
             0.9340    0.3371    0.1656    0.7482    0.1524;
             0.1299    0.1622    0.6020    0.4505    0.8258;
             0.5688    0.7943    0.2630    0.0838    0.5383;
             0.4694    0.3112    0.6541    0.2290    0.9961];

    assert(abs(alg.array_interp(array,[3 3],'cubic') - array(3,3)) < eps('single'))
    assert(abs(alg.array_interp(array,[2.5 2.5],'cubic') - 0.273426562500000) < eps('single'))
end
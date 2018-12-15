function test_imresize2p
    p_imresize = [39.5000   79.5000];
    sf = 2;
    
    p = alg.imresize2p(p_imresize, sf);
    
    % Assert
    assert(all(all(abs(p - [20 40]) < 1e-4)));
    assert(all(all(abs(p_imresize - alg.p2imresize(p, sf)) < 1e-4)));
end

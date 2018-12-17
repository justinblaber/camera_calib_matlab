function test_p2imresize
    p = [20 40];
    sf = 2;

    p_imresize = alg.p2imresize(p, sf);

    % Assert
    assert(all(all(abs(p_imresize - [39.5000   79.5000]) < 1e-4)));
    assert(all(all(abs(p - alg.imresize2p(p_imresize, sf)) < 1e-4)));
end

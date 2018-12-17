function test_asymptotes
    Aq = 1.0e+02 * [-0.000036237820288   0.001259844341262  -0.039754180081051;
                     0.001259844341262   0.000136989092504  -0.041285647453782;
                    -0.039754180081051  -0.041285647453782   2.510218598695670];

    [l1, l2] = alg.asymptotes(Aq);

    %{
    % Plot example
    f = figure;
    A = Aq(1, 1);
    B = 2*Aq(1, 2);
    C = Aq(2, 2);
    D = 2*Aq(1, 3);
    E = 2*Aq(2, 3);
    F = Aq(3, 3);
    array = zeros(62, 57);
    imshow(array, []);
    hold on;
    e1 = ezplot(@(x, y) A.*x.^2+B.*x.*y+C.*y.^2+D.*x+E.*y+F, [1 size(array, 2) 1 size(array, 1)]); %#ok<EZPLT>
    e2 = ezplot(@(x, y) l1(1).*x + l1(2).*y + l1(3), [1 size(array, 2) 1 size(array, 1)]); %#ok<EZPLT>
    e3 = ezplot(@(x, y) l2(1).*x + l2(2).*y + l2(3), [1 size(array, 2) 1 size(array, 1)]); %#ok<EZPLT>
    set(e1, 'color', 'r');
    set(e2, 'color', 'g');
    set(e3, 'color', 'g');
    pause(1);
    close(f);
    %}

    % Assert
    assert(all(all(abs(l1 - [-0.003623782028800;
                             -0.000196862044249;
                              0.112365384116166]) < 1e-4)));
    assert(all(all(abs(l2 - 1.0e+03 * [ 0.001000000000000;
                                       -0.069586340539404
                                        2.225079029539678]) < 1e-4)));
end

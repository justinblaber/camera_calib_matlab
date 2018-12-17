function test_rot2euler
    R = [0.936293363584199  -0.275095847318244   0.218350663146334; ...
         0.289629477625516   0.956425085849232  -0.036957013524625; ...
        -0.198669330795061   0.097843395007256   0.975170327201816];
    euler = alg.rot2euler(R);

    % Assert
    assert(all(abs(euler - [0.1; 0.2; 0.3]) < 1e-4));
    assert(all(all(abs(alg.euler2rot(euler) - R) < 1e-4)));
end

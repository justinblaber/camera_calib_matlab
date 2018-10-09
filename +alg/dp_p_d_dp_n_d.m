function jacob = dp_p_d_dp_n_d(p_n_ds,A)
               
    p_prime = A*[p_n_ds ones(size(p_n_ds,1),1)]';
    u_prime = p_prime(1,:);
    v_prime = p_prime(2,:);
    w_prime = p_prime(3,:);

    
    
end

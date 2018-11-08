function test_refine_stereo_params    
    % Do checker points with covariance ----------------------------------%
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    load(fullfile(tests_path,'data','refine_stereo_params_checker_cov.mat'));
    opts.height_fp = 550;
    opts.width_fp = 600;
    opts.num_targets_height = 6;
    opts.num_targets_width = 7;
    opts.target_spacing = 100;
    opts.refine_stereo_params_lambda_init = 0.010000000000000;
    opts.refine_stereo_params_lambda_factor = 10;
    opts.refine_stereo_params_it_cutoff = 200;
    opts.refine_stereo_params_norm_cutoff = 1.000000000000000e-06;
    opts.verbosity = 0;
    [params_test,cov_params_test] = alg.refine_stereo_params(params_init, ...
                                                             p_cb_p_dss, ...
                                                             idx_valids, ...
                                                             f_p_w2p_p, ...
                                                             f_dp_p_dh, ...
                                                             f_p_p2p_p_d, ...
                                                             f_dp_p_d_dargs, ...       
                                                             optimization_type, ...                                        
                                                             opts, ...
                                                             cov_cb_p_dss);
                                                         
    % Assert
    assert(all(all(abs(params_test - params) < eps('single'))));
    assert(all(all(abs(cov_params_test - cov_params) < eps('single'))));
    
    clear
    
    % Do checker points without covariance -------------------------------%
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    load(fullfile(tests_path,'data','refine_stereo_params_checker.mat'));
    opts.height_fp = 550;
    opts.width_fp = 600;
    opts.num_targets_height = 6;
    opts.num_targets_width = 7;
    opts.target_spacing = 100;
    opts.refine_stereo_params_lambda_init = 0.010000000000000;
    opts.refine_stereo_params_lambda_factor = 10;
    opts.refine_stereo_params_it_cutoff = 200;
    opts.refine_stereo_params_norm_cutoff = 1.000000000000000e-06;
    opts.verbosity = 0;
    [params_test,cov_params_test] = alg.refine_stereo_params(params_init, ...
                                                             p_cb_p_dss, ...
                                                             idx_valids, ...
                                                             f_p_w2p_p, ...
                                                             f_dp_p_dh, ...
                                                             f_p_p2p_p_d, ...
                                                             f_dp_p_d_dargs, ...       
                                                             optimization_type, ...                                        
                                                             opts);
                                                         
    % Assert
    assert(all(all(abs(params_test - params) < eps('single'))));
    assert(all(all(abs(cov_params_test - cov_params) < eps('single'))));
    
    clear
end
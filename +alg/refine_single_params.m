function [A,d,Rs,ts] = refine_single_params(A,d,Rs,ts,p_cb_p_dss,idx_valids,f_p_p_d,f_p_p_d_dargs,optimization_type,opts,cov_cb_p_dss)
    % This will compute nonlinear refinement of intrinsic and extrinsic
    % camera parameters.
            
    % Validate inputs
    util.validate_A(A);
    util.validate_p_p_d_args_f(f_p_p_d);    
    
    % Get board points in world coordinates
    p_cb_ws = alg.p_cb_w(opts);
    
    % Get number of boards
    num_boards = numel(p_cb_p_dss);
    
    % Get number of distortion params    
    num_params_d = util.num_p_p_d_d_args_f(f_p_p_d);
    
    % Get initial parameter vector. params has a length of:
    %   3 + N + 6*M
    % Where N is the number of distortion parameters and M is the number
    % of calibration boards. params has the form of: 
    %   [alpha, x_o, y_o, d_1, ... d_N, ...
    %    theta_x1, theta_y1, theta_z1, t_x1, t_y1, t_z1, ... 
    %    theta_xM, theta_yM, theta_zM, t_xM, t_yM, t_zM]
    num_params = 3 + num_params_d + 6*num_boards;
    params = zeros(num_params,1);
    
    % Do intrinsic parameters first
    params(1) = A(1,1); % alpha
    params(2) = A(1,3); % x_o
    params(3) = A(2,3); % y_o
    params(4:3+num_params_d) = d;
    
    % Cycle over extrinsics and store in params vector
    for i = 1:num_boards
        params(3 + num_params_d + 6*(i-1) + 1: ...
               3 + num_params_d + 6*(i-1) + 3) = alg.rot2euler(Rs{i});
        params(3 + num_params_d + 6*(i-1) + 4: ...
               3 + num_params_d + 6*(i-1) + 6) = ts{i};        
    end
    
    % Determine which parameters to update based on type
    idx_update = false(num_params,1);
    switch optimization_type
        case 'intrinsic'
            % Only update camera matrix
            idx_update(1:3+num_params_d) = true;
        case 'extrinsic'
            % Only update rotations and translations
            idx_update(3+num_params_d+1:end) = true;
        case 'full'
            % Update everything
            idx_update(1:end) = true;
        otherwise
            error(['Input type of: "' optimization_type '" was not recognized']);
    end
    
    % For single images, remove principle point from optimization
    if num_boards == 1
        idx_update(2:3) = false;
    end  
    
    % Perform Levenberg–Marquardt iteration(s)    
    % Initialize lambda
    lambda = opts.refine_param_lambda_init;
    % Get initial mean squared error
    mse = mean(calc_res(params, ...
                        p_cb_ws, ...
                        p_cb_p_dss, ...
                        idx_valids, ...
                        f_p_p_d, ...
                        opts).^2)*2;
    for it = 1:opts.refine_param_it_cutoff                                  
        % Store previous p and mse  
        params_prev = params;
        mse_prev = mse;
                
        % Compute delta_params
        delta_params = calc_delta_params(params_prev, ...
                                         p_cb_ws, ...
                                         p_cb_p_dss, ...
                                         idx_update, ...
                                         lambda);
        
        % update params and mse
        params(idx_update) = params_prev(idx_update) + delta_params;
        mse = mean(calc_res(params, ...
                            p_cb_ws, ...
                            p_cb_p_dss, ...
                            idx_valids, ...
                            f_p_p_d, ...
                            opts).^2)*2;
        
        % If mse decreases, decrease lambda and store results; if mse
        % increases, then increase lambda until mse descreases
        if mse < mse_prev
            % Decrease lambda and continue to next iteration
            lambda = lambda/opts.refine_param_lambda_factor;            
        else
            while mse >= mse_prev
                % Increase lambda and recompute params
                lambda = opts.refine_param_lambda_factor*lambda;      
                
                if isinf(lambda)
                    % This will already be a very, very small step, so just
                    % exit
                    delta_params(:) = 0;
                    mse = mse_prev;
                    params = params_prev;
                    break
                end
                
                % Compute delta_p
                delta_params = calc_delta_params(params_prev, ...
                                                 p_cb_ws, ...
                                                 p_cb_p_dss, ...
                                                 idx_update, ...
                                                 lambda);
            
                % update params and mse
                params(idx_update) = params_prev(idx_update) + delta_params;
                mse = mean(calc_res(params, ...
                                    p_cb_ws, ...
                                    p_cb_p_dss, ...
                                    idx_valids, ...
                                    f_p_p_d, ...
                                    opts).^2)*2;
            end            
        end
                       
        % Exit if change in distance is small
        diff_norm = norm(delta_params);         
        if strcmp(optimization_type,'full')
            disp(['Iteration #: ' sprintf('%3u',it) '; ' ...
                  'MSE: ' sprintf('%12.10f',mse) '; ' ...
                  'Norm of delta_p: ' sprintf('%12.10f',diff_norm) '; ' ...
                  'lambda: ' sprintf('%12.10f',lambda)]);
        end        
        if diff_norm < opts.refine_param_norm_cutoff
            break
        end
    end    
    if it == opts.refine_param_it_cutoff
        warning('iterations hit cutoff before converging!!!');
    end
        
    % Get outputs from params 
    A = [params(1) 0         params(3);
         0         params(2) params(4);
         0         0         1];
    d = params(5:8);  
    Rs = {};
    ts = {};
    for i = 1:num_boards
        Rs{i} = alg.euler2rot(params(8+6*(i-1)+1:8+6*(i-1)+3)); %#ok<AGROW>
        ts{i} = params(8+6*(i-1)+4:8+6*(i-1)+6); %#ok<AGROW>
    end
end

function res = calc_res(params,p_ws,p_p_dss,idx_valids,f_p_p_d,opts)   
    % Get number of distortion params    
    num_params_d = util.num_p_p_d_d_args_f(f_p_p_d);
    
    % Get intrinsic parameters
    A = [params(1) 0         params(2);
         0         params(1) params(3);
         0         0         1];
    d = params(4:3+num_params_d)';
    
    res = [];
    for i = 1:numel(p_p_dss)
        % Get rotation and translation for this board
        R = alg.euler2rot(params(3 + num_params_d + 6*(i-1) + 1: ...
                                 3 + num_params_d + 6*(i-1) + 3));
        t = params(3 + num_params_d + 6*(i-1) + 4: ...
                   3 + num_params_d + 6*(i-1) + 6);
               
        % Compute distortion model points to compute residuals
        p_p_d_ms = alg.p_p_d_m(A,d,R,t,f_p_p_d,p_ws,opts);
            
        % Store residuals - take valid indices into account
        res = vertcat(res, p_p_d_ms(idx_valids{i},1)-p_p_dss{i}(idx_valids{i},1), ...
                           p_p_d_ms(idx_valids{i},2)-p_p_dss{i}(idx_valids{i},2)); %#ok<AGROW>
    end
end

function delta_params = calc_delta_params(params,p_cb_ws,p_cb_pss,update_idx,lambda)
    % Initialize jacobian
    jacob = sparse(2*numel(p_cb_pss)*size(p_cb_ws,1),numel(params));
    
    % Get intrinsic parameters
    A = [params(1) 0         params(3);
         0         params(2) params(4);
         0         0         1];
    d = params(5:8)';

    % Fill jacobian and residuals per board
    for i = 1:numel(p_cb_pss)
        % Get rotation and translation for this board
        R = alg.euler2rot(params(8+6*(i-1)+1:8+6*(i-1)+3));
        t = params(8+6*(i-1)+4:8+6*(i-1)+6);

        % Intrinsic params
        jacob((i-1)*2*size(p_cb_ws,1)+1:i*2*size(p_cb_ws,1),1:8) = ...
            alg.dp_m_dintrinsic(A,d,R,t,p_cb_ws); %#ok<SPRIX>

        % Extrinsic params
        dR_deuler = alg.dR_deuler(alg.rot2euler(R));
        dRt_dm = blkdiag(dR_deuler(1:6,:),eye(3));
        jacob((i-1)*2*size(p_cb_ws,1)+1:i*2*size(p_cb_ws,1),8+(i-1)*6+1:8+i*6) = ...
            alg.dp_m_dextrinsic(A,d,R,t,dRt_dm,p_cb_ws); %#ok<SPRIX>
    end  

    % Get change in params using Levenberg–Marquardt update     
    delta_params = -lscov(jacob(:,update_idx)'*jacob(:,update_idx)+lambda*eye(sum(update_idx)),jacob(:,update_idx)'*calc_res(params,p_cb_ws,p_cb_pss));        
end

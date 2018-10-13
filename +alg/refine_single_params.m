function [A,d,Rs,ts] = refine_single_params(A,d,Rs,ts,p_cb_p_dss,idx_valids,f_p_w2p_p,f_dp_p_dh,f_p_p2p_p_d,f_dp_p_d_dargs,optimization_type,opts,cov_cb_p_dss)
    % This will compute nonlinear refinement of intrinsic and extrinsic
    % camera parameters.
    
    % Validate inputs
    util.validate_A(A);
    util.validate_f_p_p2p_p_d(f_p_p2p_p_d);
    
    % Get board points in world coordinates
    p_cb_ws = alg.p_cb_w(opts);
    
    % Get number of boards
    num_boards = numel(p_cb_p_dss);
    
    % Get number of distortion params    
    num_params_d = nargin(f_p_p2p_p_d) - 5;
    
    % Get initial parameter vector. params has a length of:
    %   3 + M + 6*N
    % Where M is the number of distortion parameters and N is the number
    % of calibration boards. params has the form of: 
    %   [alpha, x_o, y_o, d_1, ... d_M, ...
    %    theta_x1, theta_y1, theta_z1, t_x1, t_y1, t_z1, ... 
    %    theta_xN, theta_yN, theta_zN, t_xN, t_yN, t_zN]
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
            % Only update camera matrix and distortion params
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
    lambda = opts.refine_single_params_lambda_init;
    % Get initial mean squared error
    mse = calc_mse(params, ...
                   p_cb_ws, ...
                   p_cb_p_dss, ...
                   idx_valids, ...
                   f_p_w2p_p, ...
                   f_p_p2p_p_d);
    for it = 1:opts.refine_single_params_it_cutoff                                  
        % Store previous p and mse  
        params_prev = params;
        mse_prev = mse;
                
        % Compute delta_params
        delta_params = calc_delta_params(params_prev, ...
                                         p_cb_ws, ...
                                         p_cb_p_dss, ...
                                         idx_valids, ...
                                         f_p_w2p_p, ...
                                         f_dp_p_dh, ...
                                         f_p_p2p_p_d, ...
                                         f_dp_p_d_dargs, ...
                                         idx_update, ...
                                         lambda);
        
        % update params and mse
        params(idx_update) = params_prev(idx_update) + delta_params;
        mse = calc_mse(params, ...
                       p_cb_ws, ...
                       p_cb_p_dss, ...
                       idx_valids, ...
                       f_p_w2p_p, ...
                       f_p_p2p_p_d);
        
        % If mse decreases, decrease lambda and store results; if mse
        % increases, then increase lambda until mse descreases
        if mse < mse_prev
            % Decrease lambda and continue to next iteration
            lambda = lambda/opts.refine_single_params_lambda_factor;            
        else
            while mse >= mse_prev
                % Increase lambda and recompute params
                lambda = opts.refine_single_params_lambda_factor*lambda;      
                
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
                                                 idx_valids, ...
                                                 f_p_w2p_p, ...
                                                 f_dp_p_dh, ...
                                                 f_p_p2p_p_d, ...
                                                 f_dp_p_d_dargs, ...
                                                 idx_update, ...
                                                 lambda);
            
                % update params and mse
                params(idx_update) = params_prev(idx_update) + delta_params;
                mse = calc_mse(params, ...
                               p_cb_ws, ...
                               p_cb_p_dss, ...
                               idx_valids, ...
                               f_p_w2p_p, ...
                               f_p_p2p_p_d);
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
        if diff_norm < opts.refine_single_params_norm_cutoff
            break
        end
    end    
    if it == opts.refine_single_params_it_cutoff
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

function mse = calc_mse(params,p_ws,p_p_dss,idx_valids,f_p_w2p_p,f_p_p2p_p_d)
    % Calculate residuals
    res = calc_res(params, ...
                   p_ws, ...
                   p_p_dss, ...
                   idx_valids, ...
                   f_p_w2p_p, ...
                   f_p_p2p_p_d);
               
    % Get MSE
    mse = mean(res.^2)*2;
end

function res = calc_res(params,p_ws,p_p_dss,idx_valids,f_p_w2p_p,f_p_p2p_p_d)   
    % Get number of distortion params    
    num_params_d = nargin(f_p_p2p_p_d) - 5;
    
    % Get intrinsic parameters
    A = [params(1) 0         params(2);
         0         params(1) params(3);
         0         0         1];
    d = params(4:3 + num_params_d);
    
    % Get residuals
    res = zeros(2*sum(cellfun(@sum,idx_valids)),1);
    for i = 1:numel(p_p_dss)
        % Get rotation and translation for this board
        R = alg.euler2rot(params(3 + num_params_d + 6*(i-1) + 1: ...
                                 3 + num_params_d + 6*(i-1) + 3));
        t = params(3 + num_params_d + 6*(i-1) + 4: ...
                   3 + num_params_d + 6*(i-1) + 6);
               
        % Get homography
        H = A*[R(:,1) R(:,2) t];
        
        % Get pixel points
        p_ps = f_p_w2p_p(p_ws,H);
        
        % Get distorted pixel points
        p_p_d_ms = alg.p_p2p_p_d(p_ps,f_p_p2p_p_d,A,d);
            
        % Store residuals - take valid indices into account
        res(2*sum(cellfun(@sum,idx_valids(1:i-1)))+1:2*sum(cellfun(@sum,idx_valids(1:i)))) = ...
            reshape(vertcat((p_p_d_ms(idx_valids{i},1)-p_p_dss{i}(idx_valids{i},1))', ...
                            (p_p_d_ms(idx_valids{i},2)-p_p_dss{i}(idx_valids{i},2))'),[],1);
    end
end

function delta_params = calc_delta_params(params,p_ws,p_p_dss,idx_valids,f_p_w2p_p,f_dp_p_dh,f_p_p2p_p_d,f_dp_p_d_dargs,idx_update,lambda)
    % Get number of distortion params    
    num_params_d = nargin(f_dp_p_d_dargs{1}) - 5;
        
    % Get intrinsic parameters
    A = [params(1) 0         params(2);
         0         params(1) params(3);
         0         0         1];
    d = params(4:3+num_params_d);

    % Get jacobian
    jacob = sparse(2*sum(cellfun(@sum,idx_valids)),numel(params));
    for i = 1:numel(p_p_dss)
        % Get rotation and translation for this board
        R = alg.euler2rot(params(3 + num_params_d + 6*(i-1) + 1: ...
                                 3 + num_params_d + 6*(i-1) + 3));
        t = params(3 + num_params_d + 6*(i-1) + 4: ...
                   3 + num_params_d + 6*(i-1) + 6);
        
        % Intrinsics
        jacob(2*sum(cellfun(@sum,idx_valids(1:i-1)))+1:2*sum(cellfun(@sum,idx_valids(1:i))),1:3 + num_params_d) = ...
            alg.dp_p_d_dintrinsic(p_ws(idx_valids{i},:),f_p_w2p_p,f_dp_p_dh,R,t,f_dp_p_d_dargs,A,d); %#ok<SPRIX>

        % Extrinsics
        dr_deuler = alg.dr_deuler(alg.rot2euler(R));
        drt_dm = blkdiag(dr_deuler,eye(3));
        jacob(2*sum(cellfun(@sum,idx_valids(1:i-1)))+1:2*sum(cellfun(@sum,idx_valids(1:i))),3 + num_params_d + 6*(i-1) + 1:3 + num_params_d + 6*(i-1) + 6) = ...
            alg.dp_p_d_dextrinsic(p_ws(idx_valids{i},:),f_p_w2p_p,f_dp_p_dh,R,t,f_dp_p_d_dargs,A,d,drt_dm); %#ok<SPRIX>
    end  

    % Get change in params using Levenberg–Marquardt update     
    delta_params = -lscov(jacob(:,idx_update)'*jacob(:,idx_update)+lambda*eye(sum(idx_update)),jacob(:,idx_update)'*calc_res(params,p_ws,p_p_dss,idx_valids,f_p_w2p_p,f_p_p2p_p_d));        
end

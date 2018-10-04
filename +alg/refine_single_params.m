function [A,d,Rs,ts] = refine_single_params(A,d,Rs,ts,p_cb_p_dss,optimization_type,opts,cov_cb_p_dss)
    % This will compute nonlinear refinement of intrinsic and extrinsic
    % camera parameters.
        
    % Get board points in world coordinates
    p_cb_ws = alg.p_cb_w(opts);
    
    % Get number of boards
    num_boards = numel(p_cb_p_dss);
    
    % Supply initial parameter vector. p has a length of 8 + 6*M, where M 
    % is the number of calibration boards. There are 8 intrinsic 
    % parameters. There are 6 extrinsic parameters per board.
    % p has form of: 
    %   [alpha_x, alpha_y, x_o, y_o, beta_1, beta_2, beta_3, beta_4, ...
    %    theta_x1, theta_y1, theta_z1, t_x1, t_y1, t_z1, ... 
    %    theta_xM, theta_yM, theta_zM, t_xM, t_yM, t_zM]
    num_params = 8+6*num_boards;
    p = zeros(num_params,1);
    
    % Do intrinsic parameters first
    p(1) = A(1,1);
    p(2) = A(2,2);
    p(3) = A(1,3);
    p(4) = A(2,3);
    p(5:8) = d;
    
    % Cycle over rotations and translations and store in params vector
    for i = 1:num_boards
        p(8+6*(i-1)+1:8+6*(i-1)+3) = alg.rot2euler(Rs{i});
        p(8+6*(i-1)+4:8+6*(i-1)+6) = ts{i};        
    end
           
    % Determine which parameters to update based on type
    update_idx = false(num_params,1);
    switch optimization_type
        case 'intrinsic'
            % Only update camera matrix
            update_idx(1:8) = true;
        case 'extrinsic'
            % Only update rotations and translations
            update_idx(9:end) = true;
        case 'full'
            % Attempt to calibrate everything
            update_idx(1:end) = true;
        otherwise
            error(['Input type of: "' optimization_type '" was not recognized']);
    end
    
    % For single images, remove principle point from optimization
    if num_boards == 1
        update_idx(3:4) = false;
    end  
    
    % Perform Levenberg–Marquardt iteration(s)    
    % Initialize lambda
    lambda = opts.refine_param_lambda_init;
    % Get initial mean squared error
    mse = mean(calc_res(p,p_cb_ws,p_cb_p_dss).^2)*2;
    for it = 1:opts.refine_param_it_cutoff                                  
        % Store previous p and mse  
        p_prev = p;
        mse_prev = mse;
                
        % Compute delta_p
        delta_p = calc_delta_p(p_prev, ...
                               p_cb_ws, ...
                               p_cb_p_dss, ...
                               update_idx, ...
                               lambda);
        
        % update params and mse
        p(update_idx) = p_prev(update_idx) + delta_p;
        mse = mean(calc_res(p,p_cb_ws,p_cb_p_dss).^2)*2;
        
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
                    delta_p(:) = 0;
                    mse = mse_prev;
                    p = p_prev;
                    break
                end
                
                % Compute delta_p
                delta_p = calc_delta_p(p_prev, ...
                                       p_cb_ws, ...
                                       p_cb_p_dss, ...
                                       update_idx, ...
                                       lambda);
            
                % update params and mse
                p(update_idx) = p_prev(update_idx) + delta_p;
                mse = mean(calc_res(p,p_cb_ws,p_cb_p_dss).^2)*2;
            end            
        end
                       
        % Exit if change in distance is small
        diff_norm = norm(delta_p);         
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
        
    % Get outputs from p  
    A = [p(1) 0    p(3);
         0    p(2) p(4);
         0    0    1];
    d = p(5:8);  
    Rs = {};
    ts = {};
    for i = 1:num_boards
        Rs{i} = alg.euler2rot(p(8+6*(i-1)+1:8+6*(i-1)+3)); %#ok<AGROW>
        ts{i} = p(8+6*(i-1)+4:8+6*(i-1)+6); %#ok<AGROW>
    end
end

function res = calc_res(p,p_cb_ws,p_cb_pss)    
    res = zeros(2*numel(p_cb_pss)*size(p_cb_ws,1),1);
    
    % Get intrinsic parameters
    A = [p(1) 0    p(3);
         0    p(2) p(4);
         0    0    1];
    distortion = p(5:8)';
    
    for i = 1:numel(p_cb_pss)
        % Get rotation and translation for this board
        R = alg.euler2rot(p(8+6*(i-1)+1:8+6*(i-1)+3));
        t = p(8+6*(i-1)+4:8+6*(i-1)+6);
        
        % Compute model points to compute residuals
        p_m = alg.p_m(A,distortion,R,t,p_cb_ws);
            
        % Store residuals
        res((i-1)*2*size(p_cb_ws,1)+1:i*2*size(p_cb_ws,1)) =  ...
            vertcat(p_m(:,1)-p_cb_pss{i}(:,1), ...
                    p_m(:,2)-p_cb_pss{i}(:,2));           
    end
end

function delta_p = calc_delta_p(p,p_cb_ws,p_cb_pss,update_idx,lambda)
    % Initialize jacobian
    jacob = sparse(2*numel(p_cb_pss)*size(p_cb_ws,1),numel(p));
    
    % Get intrinsic parameters
    A = [p(1) 0    p(3);
         0    p(2) p(4);
         0    0    1];
    distortion = p(5:8)';

    % Fill jacobian and residuals per board
    for i = 1:numel(p_cb_pss)
        % Get rotation and translation for this board
        R = alg.euler2rot(p(8+6*(i-1)+1:8+6*(i-1)+3));
        t = p(8+6*(i-1)+4:8+6*(i-1)+6);

        % Intrinsic params
        jacob((i-1)*2*size(p_cb_ws,1)+1:i*2*size(p_cb_ws,1),1:8) = ...
            alg.dp_m_dintrinsic(A,distortion,R,t,p_cb_ws); %#ok<SPRIX>

        % Extrinsic params
        dR_deuler = alg.dR_deuler(alg.rot2euler(R));
        dRt_dm = blkdiag(dR_deuler(1:6,:),eye(3));
        jacob((i-1)*2*size(p_cb_ws,1)+1:i*2*size(p_cb_ws,1),8+(i-1)*6+1:8+i*6) = ...
            alg.dp_m_dextrinsic(A,distortion,R,t,dRt_dm,p_cb_ws); %#ok<SPRIX>
    end  

    % Get change in params using Levenberg–Marquardt update     
    delta_p = -lscov(jacob(:,update_idx)'*jacob(:,update_idx)+lambda*eye(sum(update_idx)),jacob(:,update_idx)'*calc_res(p,p_cb_ws,p_cb_pss));        
end

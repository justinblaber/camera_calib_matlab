function [A,distortion,rotations,translations] = refine_single_params(A,distortion,rotations,translations,board_points_ps,type,calib_config)
    % This will compute nonlinear refinement of intrinsic and extrinsic
    % camera parameters.
    %
    % Inputs:    
    %	A - array; initial guess of camera matrix containing:
    %           [alpha    0       x_o;
    %            0        alpha   y_o;
    %            0        0       1]
    %   distortion - array; initial guess of distortions (radial and 
    %       tangential) stored as: 
    %       [beta1; beta2; beta3; beta4]  
    %   rotations - cell; Mx1 initial guesses of rotations
    %   translations - cell; Mx1 initial guesses of translations
    %   board_points_ps - cell; Mx1 of optimized subpixel calibration 
    %       board points in pixel coordinates.
    %   type - string; 
    %       'extrinsic' - Only rotations and translations are optimized
    %       'intrisic' - Only camera parameters (A and distortion) are 
    %           optimized
    %       'full' - Attempts to do full calibration
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %	A - array; optimized camera matrix
    %   distortion - array; optimized distortions (radial and 
    %       tangential) stored as: 
    %       [beta1; beta2; beta3; beta4]  
    %   rotations - cell; Mx1 optimized rotations
    %   translations - cell; Mx1 optimized translations
          
    % TODO: make sure rotations and translations have the same length
        
    if calib_config.verbose > 1    
        disp('---');
    end
    
    % Get board points in world coordinates
    board_points_w = alg.cb_points(calib_config);
    
    % Get number of boards
    num_boards = length(board_points_ps);
    
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
    p(5:8) = distortion;
    
    % Cycle over rotations and translations and store in params vector
    for i = 1:num_boards
        p(8+6*(i-1)+1:8+6*(i-1)+3) = alg.rot2euler(rotations{i});
        p(8+6*(i-1)+4:8+6*(i-1)+6) = translations{i};        
    end
           
    % Determine which parameters to update based on type
    update_idx = false(num_params,1);
    switch type
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
            error(['Input type of: "' type '" was not recognized']);
    end
    
    % For single images, remove principle point from optimization
    if num_boards == 1
        update_idx(3:4) = false;
    end  
    
    % Perform Levenberg–Marquardt iteration(s)    
    % Initialize lambda
    lambda = calib_config.refine_param_lambda_init;
    % Get initial mean squared error
    mse = mean(calc_res(p,board_points_w,board_points_ps).^2)*2;
    for it = 1:calib_config.refine_param_it_cutoff                                  
        % Store previous p and mse  
        p_prev = p;
        mse_prev = mse;
                
        % Compute delta_p
        delta_p = calc_delta_p(p_prev, ...
                               board_points_w, ...
                               board_points_ps, ...
                               update_idx, ...
                               lambda);
        
        % update params and mse
        p(update_idx) = p_prev(update_idx) + delta_p;
        mse = mean(calc_res(p,board_points_w,board_points_ps).^2)*2;
        
        % If mse decreases, decrease lambda and store results; if mse
        % increases, then increase lambda until mse descreases
        if mse < mse_prev
            % Decrease lambda and continue to next iteration
            lambda = lambda/calib_config.refine_param_lambda_factor;            
        else
            while mse >= mse_prev
                % Increase lambda and recompute params
                lambda = calib_config.refine_param_lambda_factor*lambda;      
                
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
                                       board_points_w, ...
                                       board_points_ps, ...
                                       update_idx, ...
                                       lambda);
            
                % update params and mse
                p(update_idx) = p_prev(update_idx) + delta_p;
                mse = mean(calc_res(p,board_points_w,board_points_ps).^2)*2;
            end            
        end
                       
        % Exit if change in distance is small
        diff_norm = norm(delta_p);         
        if calib_config.verbose > 1 || strcmp(type,'full')
            disp(['Iteration #: ' sprintf('%3u',it) '; ' ...
                  'MSE: ' sprintf('%12.10f',mse) '; ' ...
                  'Norm of delta_p: ' sprintf('%12.10f',diff_norm) '; ' ...
                  'lambda: ' sprintf('%12.10f',lambda)]);
        end        
        if diff_norm < calib_config.refine_param_norm_cutoff
            break
        end
    end    
    if it == calib_config.refine_param_it_cutoff
        warning('iterations hit cutoff before converging!!!');
    end
        
    % Get outputs from p  
    A = [p(1) 0    p(3);
         0    p(2) p(4);
         0    0    1];
    distortion = p(5:8);  
    rotations = {};
    translations = {};
    for i = 1:num_boards
        rotations{i} = alg.euler2rot(p(8+6*(i-1)+1:8+6*(i-1)+3)); %#ok<AGROW>
        translations{i} = p(8+6*(i-1)+4:8+6*(i-1)+6); %#ok<AGROW>
    end
end

function res = calc_res(p,board_points_w,board_points_ps)    
    res = zeros(2*length(board_points_ps)*size(board_points_w,1),1);
    
    % Get intrinsic parameters
    A = [p(1) 0    p(3);
         0    p(2) p(4);
         0    0    1];
    distortion = p(5:8)';
    
    for i = 1:length(board_points_ps)
        % Get rotation and translation for this board
        R = alg.euler2rot(p(8+6*(i-1)+1:8+6*(i-1)+3));
        t = p(8+6*(i-1)+4:8+6*(i-1)+6);
        
        % Compute model points to compute residuals
        p_m = alg.p_m(A,distortion,R,t,board_points_w);
            
        % Store residuals
        res((i-1)*2*size(board_points_w,1)+1:i*2*size(board_points_w,1)) =  ...
            vertcat(p_m(:,1)-board_points_ps{i}(:,1), ...
                    p_m(:,2)-board_points_ps{i}(:,2));           
    end
end

function delta_p = calc_delta_p(p,board_points_w,board_points_ps,update_idx,lambda)
    % Initialize jacobian
    jacob = sparse(2*length(board_points_ps)*size(board_points_w,1),length(p));
    
    % Get intrinsic parameters
    A = [p(1) 0    p(3);
         0    p(2) p(4);
         0    0    1];
    distortion = p(5:8)';

    % Fill jacobian and residuals per board
    for i = 1:length(board_points_ps)
        % Get rotation and translation for this board
        R = alg.euler2rot(p(8+6*(i-1)+1:8+6*(i-1)+3));
        t = p(8+6*(i-1)+4:8+6*(i-1)+6);

        % Intrinsic params
        jacob((i-1)*2*size(board_points_w,1)+1:i*2*size(board_points_w,1),1:8) = ...
            alg.dp_m_dintrinsic(A,distortion,R,t,board_points_w); %#ok<SPRIX>

        % Extrinsic params
        dR_deuler = alg.dR_deuler(alg.rot2euler(R));
        dRt_dm = blkdiag(dR_deuler(1:6,:),eye(3));
        jacob((i-1)*2*size(board_points_w,1)+1:i*2*size(board_points_w,1),8+(i-1)*6+1:8+i*6) = ...
            alg.dp_m_dextrinsic(A,distortion,R,t,dRt_dm,board_points_w); %#ok<SPRIX>
    end  

    % Get change in params using Levenberg–Marquardt update     
    delta_p = -mldivide(jacob(:,update_idx)'*jacob(:,update_idx)+lambda*eye(sum(update_idx)),jacob(:,update_idx)'*calc_res(p,board_points_w,board_points_ps));        
end
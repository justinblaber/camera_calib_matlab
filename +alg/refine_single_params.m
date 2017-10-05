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
    
    % Get number of boards and number of points
    num_boards = length(board_points_ps);
    num_points = size(board_points_w,1);
    
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
   
    % Initialize jacobian
    jacob = sparse(2*num_boards*num_points,num_params);
    res = zeros(2*num_boards*num_points,1);
        
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
    
    % Perform gauss newton iteration(s)    
    for it = 1:calib_config.refine_param_it_cutoff        
        % Get intrinsic parameters
        A = [p(1) 0    p(3);
             0    p(2) p(4);
             0    0    1];
        distortion = p(5:8)';
                    
        % Fill jacobian and residuals per board
        for i = 1:num_boards
            % Get rotation and translation for this board
            R = alg.euler2rot(p(8+6*(i-1)+1:8+6*(i-1)+3));
            t = p(8+6*(i-1)+4:8+6*(i-1)+6);
            
            % Intrinsic params
            jacob((i-1)*2*num_points+1:i*2*num_points,1:8) = alg.dp_m_dintrinsic(A, ...
                                                                                 distortion, ...
                                                                                 R, ...
                                                                                 t, ...
                                                                                 board_points_w); %#ok<SPRIX>
            
            % Extrinsic params
            dR_deuler = alg.dR_deuler(alg.rot2euler(R));
            dRt_dm = blkdiag(dR_deuler(1:6,:),eye(3));
            jacob((i-1)*2*num_points+1:i*2*num_points,8+(i-1)*6+1:8+i*6) = alg.dp_m_dextrinsic(A, ...
                                                                                               distortion, ...
                                                                                               R, ...
                                                                                               t, ...
                                                                                               dRt_dm, ...
                                                                                               board_points_w); %#ok<SPRIX>
            
            % Compute model points to compute residuals
            p_m = alg.p_m(A, ...
                          distortion, ...
                          R, ...
                          t, ...
                          board_points_w);
            
            % Store residuals
            res((i-1)*2*num_points+1:i*2*num_points) = vertcat(p_m(:,1)-board_points_ps{i}(:,1), ...
                                                               p_m(:,2)-board_points_ps{i}(:,2));           
        end  
                
        % Get and store update
        delta_p = -inv(jacob(:,update_idx)'*jacob(:,update_idx))*jacob(:,update_idx)'*res;
        p(update_idx) = p(update_idx) + delta_p;
        
        % Store norm of residual
        norm_res = norm(res);
        
        % Exit if change in distance is small
        norm_delta_p = norm(delta_p);          
        if calib_config.verbose > 1 
            disp(['Iteration #: ' num2str(it)]);
            disp(['Difference norm for nonlinear parameter refinement: ' num2str(norm_delta_p)]);
            disp(['Norm of residual: ' num2str(norm_res)]);
        end
        if norm(delta_p) < calib_config.refine_param_norm_cutoff
            break
        end
    end    
    if calib_config.verbose > 1 && it == calib_config.refine_param_it_cutoff
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
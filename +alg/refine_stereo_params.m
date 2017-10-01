function [A,distortion,rotations,translations,R_s,t_s] = refine_stereo_params(A,distortion,rotations,translations,board_points_ps,R_s,t_s,type,calib_config)
    % This will compute nonlinear refinement of intrinsic and extrinsic
    % camera parameters for both left and right cameras.
    %
    % Inputs: 
    %   A - struct; contains:
    %       .L - array; camera matrix for the left camera
    %       .R - array; camera matrix for the right camera
    %   distortion - struct; contains:
    %       .L - array; 4x1 array of distortions for the left camera
    %       .R - array; 4x1 array of distortions for the right camera
    %       stored as: 
    %           [beta1; beta2; beta3; beta4]  
    %   rotations - struct; contains:
    %       .L - cell; rotations for the left camera
    %       .R - cell; rotations for the right camera
    %   translations - struct; contains:
    %       .L - cell; translations for the left camera
    %       .R - cell; translations for the right camera
    %   board_points_ps - struct; contains:
    %       .L - cell; cell array of calibration board points for the left 
    %           camera.
    %       .R - cell; cell array of calibration board points for the right
    %           camera.
    %   R_s - array; 3x3 rotation matrix describing rotation from the left
    %       camera to the right camera
    %   t_s - array; 3x1 translation vector describing translation from the
    %       left camera to the right camera
    %   type - string; 
    %       'full' - Attempts to do full calibration
    %   calib_config - struct; this is the struct returned by
    %       util.load_calib_config()
    %
    % Outputs:
    %   A - struct; contains:
    %       .L - array; optimized camera matrix for the left camera
    %       .R - array; optimized camera matrix for the right camera
    %   distortion - struct; contains:
    %       .L - array; 4x1 array of optimized distortions for the left
    %           camera
    %       .R - array; 4x1 array of optimized distortions for the right
    %           camera
    %       stored as: 
    %           [beta1; beta2; beta3; beta4]  
    %   rotations - struct; contains:
    %       .L - cell; optimized rotations for the left camera
    %       .R - cell; optimized rotations for the right camera
    %   translations - struct; contains:
    %       .L - cell; optimized translations for the left camera
    %       .R - cell; optimized translations for the right camera
    %   R_s - array; 3x3 rotation matrix describing rotation from the left
    %       camera to the right camera
    %   t_s - array; 3x1 translation vector describing translation from the
    %       left camera to the right camera
    
    disp('---');
              
    % Get board points in world coordinates
    board_points_w = alg.cb_points(calib_config);
    
    % Get number of boards, points, and parameters
    num_boards = length(board_points_ps.L);
    num_points = size(board_points_w,1);
    
    % Supply initial parameter vector. p has a length of 22 + 6*M, where M 
    % is the number of calibration boards. There are 16 intrinsic 
    % parameters (8 for the left camera and 8 for the right camera) and
    % 6*M+6 extrinsic parameters total.
    % p has form of: 
    %   [alpha_Lx, alpha_Ly, x_Lo, y_Lo, beta_L1, beta_L2, beta_L3, beta_L4, ...
    %    alpha_Rx, alpha_Ry, x_Ro, y_Ro, beta_R1, beta_R2, beta_R3, beta_R4, ...
    %    theta_Lx1, theta_Ly1, theta_Lz1, t_Lx1, t_Ly1, t_Lz1, ... 
    %    theta_LxM, theta_LyM, theta_LzM, t_LxM, t_LyM, t_LzM, ...
    %    theta_sx, theta_sy, theta_sz, t_sx, t_sy, t_sz]
    num_params = 22+6*num_boards;
    p = zeros(num_params,1);

    % Do extrinsic parameters first
    % Left
    p(1) = A.L(1,1);
    p(2) = A.L(2,2);
    p(3) = A.L(1,3);
    p(4) = A.L(2,3);
    p(5:8) = distortion.L;
    % Right
    p(9) = A.R(1,1);
    p(10) = A.R(2,2);
    p(11) = A.R(1,3);
    p(12) = A.R(2,3);
    p(13:16) = distortion.R;
    
    % Cycle over rotations and translations and store params. Note that
    % rotations.R and translations.R are not used.
    for i = 1:num_boards     
        p(16+6*(i-1)+1:16+6*(i-1)+3) = alg.rot2euler(rotations.L{i});
        p(16+6*(i-1)+4:16+6*(i-1)+6) = translations.L{i};        
    end
    % Store R_s and t_s
    p(16+6*num_boards+1:16+6*num_boards+3) = alg.rot2euler(R_s);
    p(16+6*num_boards+4:16+6*num_boards+6) = t_s;
   
    % Initialize jacobian
    jacob = sparse(4*num_boards*num_points,num_params);
    res = zeros(4*num_boards*num_points,1);
    
    % TODO: Possibly add other optimization options besides "full"
    
    % Determine which parameters to update based on type
    update_idx = false(num_params,1);
    switch type
        case 'full'
            % Attempt to calibrate everything
            update_idx(1:end) = true;
        otherwise
            error(['Input type of: "' type '" was not recognized']);
    end    
    
    % For single images, remove principle points from optimization
    if num_boards == 1
        update_idx(3:4) = false;
        update_idx(11:12) = false;
    end  
    
    % Perform gauss newton iteration(s)    
    for it = 1:calib_config.refine_param_it_cutoff      
        % Get intrinsic parameters
        % left
        A_L = [p(1) 0    p(3);
               0    p(2) p(4);
               0    0    1];
        distortion_L = p(5:8)';
        % right
        A_R = [p(9) 0     p(11);
               0    p(10) p(12);
               0    0     1];
        distortion_R = p(13:16)';
        
        % Get R_s and t_s; only need to do this once per iteration
        R_s = alg.euler2rot(p(16+6*num_boards+1:16+6*num_boards+3));
        t_s = p(16+6*num_boards+4:16+6*num_boards+6);
        
        % Fill jacobian and residuals per board
        for i = 1:num_boards
            % Fill jacobian for left board -------------------------------%
            % This is basically the same for single board calibration since
            % the left board is independent
            
            % Get rotation and translation for the left board
            R_L = alg.euler2rot(p(16+6*(i-1)+1:16+6*(i-1)+3));
            t_L = p(16+6*(i-1)+4:16+6*(i-1)+6);
            
            % Intrinsic params
            jacob((i-1)*4*num_points+1:((i-1)*4+2)*num_points,1:8) = alg.dp_m_dintrinsic(A_L, ...
                                                                                         distortion_L, ...
                                                                                         R_L, ...
                                                                                         t_L, ...
                                                                                         board_points_w); %#ok<SPRIX>
            
            % Extrinsic params
            dR_L_deuler_L = alg.dR_deuler(alg.rot2euler(R_L));
            dRt_L_dm_L = blkdiag(dR_L_deuler_L(1:6,:),eye(3));
            jacob((i-1)*4*num_points+1:((i-1)*4+2)*num_points,16+(i-1)*6+1:16+i*6) = alg.dp_m_dextrinsic(A_L, ...
                                                                                                         distortion_L, ...
                                                                                                         R_L, ...
                                                                                                         t_L, ...
                                                                                                         dRt_L_dm_L, ...
                                                                                                         board_points_w);  %#ok<SPRIX>
             
            % Compute model points to compute residuals
            p_m_L = alg.p_m(A_L, ...
                            distortion_L, ...
                            R_L, ...
                            t_L, ...
                            board_points_w);
                                        
            % Store residuals
            res((i-1)*4*num_points+1:((i-1)*4+2)*num_points) = vertcat(p_m_L(:,1)-board_points_ps.L{i}(:,1), ...
                                                                       p_m_L(:,2)-board_points_ps.L{i}(:,2)); 
            
            % Fill jacobian for right board ------------------------------%
            % Right board is dependent on left board transformation and R_s
            % and t_s, so there are some differences here.
                        
            % Get rotation and translation for the right board
            R_R = R_s*R_L;         
            t_R = R_s*t_L+t_s;
                        
            % Intrinsic params
            jacob(((i-1)*4+2)*num_points+1:i*4*num_points,9:16) = alg.dp_m_dintrinsic(A_R, ...
                                                                                      distortion_R, ...
                                                                                      R_R, ...
                                                                                      t_R, ...
                                                                                      board_points_w); %#ok<SPRIX>
                        
            % Extrinsic params; do dRt_R_dm_L first
            dRt_R_dRt_L = blkdiag(R_s,R_s,R_s);
            dRt_R_dm_L = dRt_R_dRt_L*dRt_L_dm_L;
            jacob(((i-1)*4+2)*num_points+1:i*4*num_points,16+(i-1)*6+1:16+i*6) = alg.dp_m_dextrinsic(A_R, ...
                                                                                                     distortion_R, ...
                                                                                                     R_R, ...
                                                                                                     t_R, ...
                                                                                                     dRt_R_dm_L, ...
                                                                                                     board_points_w);  %#ok<SPRIX>
            
            % Do dRt_R_dm_s next
            dRt_R_dRt_s = [R_L(1,1)*eye(3) R_L(2,1)*eye(3) R_L(3,1)*eye(3) zeros(3);
                           R_L(1,2)*eye(3) R_L(2,2)*eye(3) R_L(3,2)*eye(3) zeros(3);
                           t_L(1)*eye(3)   t_L(2)*eye(3)   t_L(3)*eye(3)   eye(3)]; 
            dR_s_deuler_s = alg.dR_deuler(alg.rot2euler(R_s));
            dRt_s_dm_s = blkdiag(dR_s_deuler_s,eye(3));
            dRt_R_dm_s = dRt_R_dRt_s*dRt_s_dm_s;
            jacob(((i-1)*4+2)*num_points+1:i*4*num_points,16+num_boards*6+1:16+(num_boards+1)*6) = alg.dp_m_dextrinsic(A_R, ...
                                                                                                                       distortion_R, ...
                                                                                                                       R_R, ...
                                                                                                                       t_R, ...
                                                                                                                       dRt_R_dm_s, ...
                                                                                                                       board_points_w);  %#ok<SPRIX>
                      
                       
            % Compute model points to compute residuals
            p_m_R = alg.p_m(A_R, ...
                            distortion_R, ...
                            R_R, ...
                            t_R, ...
                            board_points_w); 
                                        
            % Store residuals
            res(((i-1)*4+2)*num_points+1:i*4*num_points) = vertcat(p_m_R(:,1)-board_points_ps.R{i}(:,1), ...
                                                                   p_m_R(:,2)-board_points_ps.R{i}(:,2)); 
        end  
                
        % Get and store update
        delta_p = -inv(jacob(:,update_idx)'*jacob(:,update_idx))*jacob(:,update_idx)'*res;
        p(update_idx) = p(update_idx) + delta_p;
        
        % Store norm of residual
        norm_res = norm(res);
        
        % Exit if change in distance is small
        norm_delta_p = norm(delta_p);   
        disp(['Iteration #: ' num2str(it)]);
        disp(['Difference norm for nonlinear parameter refinement: ' num2str(norm_delta_p)]);
        disp(['Norm of residual: ' num2str(norm_res)]);
        if norm(delta_p) < calib_config.refine_param_norm_cutoff
            break
        end
    end    
    if it == calib_config.refine_param_it_cutoff
        warning('iterations hit cutoff before converging!!!');
    end
        
    % Get outputs from p  
    A.L = [p(1)     0       p(3);
           0        p(2)    p(4);
           0        0       1];
    A.R = [p(9)     0       p(11);
           0        p(10)   p(12);
           0        0       1];
    distortion.L = p(5:8);
    distortion.R = p(13:16); 
    
    R_s = alg.euler2rot(p(16+6*num_boards+1:16+6*num_boards+3));
    t_s = p(16+6*num_boards+4:16+6*num_boards+6);
    
    rotations.L = {};
    rotations.R = {};
    translations.L = {};
    translations.R = {};
    for i = 1:num_boards
        rotations.L{i} = alg.euler2rot(p(16+6*(i-1)+1:16+6*(i-1)+3));
        translations.L{i} = p(16+6*(i-1)+4:16+6*(i-1)+6);
        
        % Recompute R_R and t_R using R_s and t_s
        rotations.R{i} = R_s*rotations.L{i};  
        translations.R{i} = R_s*translations.L{i} + t_s;
    end  
end

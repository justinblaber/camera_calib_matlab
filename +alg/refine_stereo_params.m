function [rotations,translations,A,distortion,R_s,t_s] = refine_stereo_params(rotations,translations,A,distortion,R_s,t_s,board_points_is,type,cb_config)
              
    % Get number of calibration boards and number of board points
    board_points_w = alg.cb_points(cb_config);
    X_s = board_points_w(:,1);
    Y_s = board_points_w(:,2);
    
    % Get number of boards, points, and parameters
    num_boards = length(board_points_is.L);
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
    
    % Determine which parameters to update based on type
    update_idx = false(num_params,1);
    switch type
        case 'full'
            % Attempt to calibrate everything
            update_idx(1:num_params) = true;
        otherwise
            error(['Input type of: "' type '" was not recognized']);
    end
    
    % Perform gauss newton iteration(s)    
    for it = 1:cb_config.refine_param_it_cutoff      
        % Get intrinsic parameters
        % left
        alpha_Lx = p(1);
        alpha_Ly = p(2);
        x_Lo = p(3);
        y_Lo = p(4);        
        beta_L1 = p(5);
        beta_L2 = p(6);
        beta_L3 = p(7);
        beta_L4 = p(8);
        % right
        alpha_Rx = p(9);
        alpha_Ry = p(10);
        x_Ro = p(11);
        y_Ro = p(12);
        beta_R1 = p(13);
        beta_R2 = p(14);
        beta_R3 = p(15);
        beta_R4 = p(16);
        
        % Get R_s and t_s; only need to do this once per iteration
        euler_s = p(16+6*num_boards+1:16+6*num_boards+3);
        theta_sx = euler_s(1);
        theta_sy = euler_s(2);
        theta_sz = euler_s(3);
        R_s = alg.euler2rot(euler_s);
        t_s = p(16+6*num_boards+4:16+6*num_boards+6);
        
        % Fill jacobian and residuals per board
        for i = 1:num_boards
            % Get rotation and translation for the left board
            euler_L = p(16+6*(i-1)+1:16+6*(i-1)+3);
            theta_Lx = euler_L(1);
            theta_Ly = euler_L(2);
            theta_Lz = euler_L(3);
            R_L = alg.euler2rot(euler_L);
            t_L = p(16+6*(i-1)+4:16+6*(i-1)+6);
            
            % Get rotation and translation for the right board
            R_R = R_s*R_L;         
            t_R = R_s*t_L+t_s;
            
            % Fill jacobian for left board -------------------------------%
            % This is basically the same for single board calibration since
            % left board is "independent"
            
            % Apply xform to points to get into camera coordinates
            points_Ls = [R_L(:,1) R_L(:,2) t_L] * [board_points_w ones(size(board_points_w,1),1)]';
            x_Ls = points_Ls(1,:)';
            y_Ls = points_Ls(2,:)';
            z_Ls = points_Ls(3,:)';

            % Get normalized coordinates
            x_Ln = x_Ls./z_Ls;
            y_Ln = y_Ls./z_Ls;
            r_Ln = sqrt(x_Ln.^2 + y_Ln.^2);           
            
            % Rows of jacobian and residuals
            x_top_L = (i-1)*4*num_points+1;
            x_bottom_L = x_top_L+num_points-1;
            y_top_L = x_bottom_L+1;
            y_bottom_L = y_top_L+num_points-1;
            
            % Intrinsic params
            % x coords
            jacob(x_top_L:x_bottom_L,1) = x_Ln.*(1+beta_L1*r_Ln.^2+beta_L2*r_Ln.^4) + 2*beta_L3*x_Ln.*y_Ln + beta_L4*(r_Ln.^2 + 2*x_Ln.^2); %#ok<*SPRIX>
            jacob(x_top_L:x_bottom_L,2) = 0;
            jacob(x_top_L:x_bottom_L,3) = 1;
            jacob(x_top_L:x_bottom_L,4) = 0;
            jacob(x_top_L:x_bottom_L,5) = alpha_Lx*x_Ln.*r_Ln.^2;
            jacob(x_top_L:x_bottom_L,6) = alpha_Lx*x_Ln.*r_Ln.^4;
            jacob(x_top_L:x_bottom_L,7) = 2*alpha_Lx.*x_Ln.*y_Ln;
            jacob(x_top_L:x_bottom_L,8) = alpha_Lx*(r_Ln.^2+2*x_Ln.^2);
            
            % y coords
            jacob(y_top_L:y_bottom_L,1) = 0;
            jacob(y_top_L:y_bottom_L,2) = y_Ln.*(1+beta_L1*r_Ln.^2+beta_L2*r_Ln.^4) + beta_L3*(r_Ln.^2 + 2*y_Ln.^2) + 2*beta_L4*x_Ln.*y_Ln;
            jacob(y_top_L:y_bottom_L,3) = 0;
            jacob(y_top_L:y_bottom_L,4) = 1;
            jacob(y_top_L:y_bottom_L,5) = alpha_Ly*y_Ln.*r_Ln.^2;
            jacob(y_top_L:y_bottom_L,6) = alpha_Ly*y_Ln.*r_Ln.^4;
            jacob(y_top_L:y_bottom_L,7) = alpha_Ly*(r_Ln.^2+2*y_Ln.^2);
            jacob(y_top_L:y_bottom_L,8) = 2*alpha_Ly*x_Ln.*y_Ln;
            
            % Extrinsic params
            dx_Ln_dRt_L = [X_s./z_Ls zeros(num_points,1) -(X_s.*x_Ls)./z_Ls.^2 Y_s./z_Ls zeros(num_points,1) -(Y_s.*x_Ls)./z_Ls.^2 1./z_Ls zeros(num_points,1) -x_Ls./z_Ls.^2];
            dy_Ln_dRt_L = [zeros(num_points,1) X_s./z_Ls -(X_s.*y_Ls)./z_Ls.^2 zeros(num_points,1) Y_s./z_Ls -(Y_s.*y_Ls)./z_Ls.^2 zeros(num_points,1) 1./z_Ls -y_Ls./z_Ls.^2];
             
            % m = [theta_x theta_y theta_z tx ty tz]
            dRt_L_dm_L = [0                                                                        -sin(theta_Ly)*cos(theta_Lz)                -cos(theta_Ly)*sin(theta_Lz)                                             0   0   0;
                          0                                                                        -sin(theta_Ly)*sin(theta_Lz)                 cos(theta_Ly)*cos(theta_Lz)                                             0   0   0;
                          0                                                                        -cos(theta_Ly)                               0                                                                       0   0   0;
                          sin(theta_Lx)*sin(theta_Lz)+cos(theta_Lx)*sin(theta_Ly)*cos(theta_Lz)     sin(theta_Lx)*cos(theta_Ly)*cos(theta_Lz)  -cos(theta_Lx)*cos(theta_Lz)-sin(theta_Lx)*sin(theta_Ly)*sin(theta_Lz)   0   0   0;
                         -sin(theta_Lx)*cos(theta_Lz)+cos(theta_Lx)*sin(theta_Ly)*sin(theta_Lz)     sin(theta_Lx)*cos(theta_Ly)*sin(theta_Lz)  -cos(theta_Lx)*sin(theta_Lz)+sin(theta_Lx)*sin(theta_Ly)*cos(theta_Lz)   0   0   0;
                          cos(theta_Lx)*cos(theta_Ly)                                              -sin(theta_Lx)*sin(theta_Ly)                 0                                                                       0   0   0;
                          0                                                                         0                                           0                                                                       1   0   0;
                          0                                                                         0                                           0                                                                       0   1   0;
                          0                                                                         0                                           0                                                                       0   0   1];
            
            % Get jacobian of normalized coords
            dx_Ln_dm_L = dx_Ln_dRt_L*dRt_L_dm_L;
            dy_Ln_dm_L = dy_Ln_dRt_L*dRt_L_dm_L;
            dr_Ln_dm_L = (x_Ln.*dx_Ln_dm_L + y_Ln.*dy_Ln_dm_L)./r_Ln;
            
            % Store jacobian of pixel coords
            jacob(x_top_L:x_bottom_L,16+(i-1)*6+1:16+i*6) = alpha_Lx*(dx_Ln_dm_L.*(1+beta_L1*r_Ln.^2+beta_L2*r_Ln.^4)+x_Ln.*(2*beta_L1*r_Ln.*dr_Ln_dm_L+4*beta_L2*r_Ln.^3.*dr_Ln_dm_L)+2*beta_L3*(dx_Ln_dm_L.*y_Ln+x_Ln.*dy_Ln_dm_L)+beta_L4*(2*r_Ln.*dr_Ln_dm_L+4*x_Ln.*dx_Ln_dm_L));           
            jacob(y_top_L:y_bottom_L,16+(i-1)*6+1:16+i*6) = alpha_Ly*(dy_Ln_dm_L.*(1+beta_L1*r_Ln.^2+beta_L2*r_Ln.^4)+y_Ln.*(2*beta_L1*r_Ln.*dr_Ln_dm_L+4*beta_L2*r_Ln.^3.*dr_Ln_dm_L)+beta_L3*(2*r_Ln.*dr_Ln_dm_L+4*y_Ln.*dy_Ln_dm_L)+2*beta_L4*(dx_Ln_dm_L.*y_Ln+x_Ln.*dy_Ln_dm_L)); 
                       
            % Compute projected points from model
            x_model_L = alpha_Lx*(x_Ln.*(1+beta_L1*r_Ln.^2+beta_L2*r_Ln.^4) + 2*beta_L3*x_Ln.*y_Ln + beta_L4*(r_Ln.^2+2*x_Ln.^2)) + x_Lo;
            y_model_L = alpha_Ly*(y_Ln.*(1+beta_L1*r_Ln.^2+beta_L2*r_Ln.^4) + beta_L3*(r_Ln.^2+2*y_Ln.^2) + 2*beta_L4*x_Ln.*y_Ln) + y_Lo;     
            
            % Store residuals
            board_points_Li = board_points_is.L{i};
            res(x_top_L:x_bottom_L) = x_model_L - board_points_Li(:,1);
            res(y_top_L:y_bottom_L) = y_model_L - board_points_Li(:,2);   
            
            % Fill jacobian for right board ------------------------------%
            % Right board is dependent on left board transformation, so
            % there are some modifications here.
            
            % Apply xform to points to get into camera coordinates
            points_Rs = [R_R(:,1) R_R(:,2) t_R] * [board_points_w ones(size(board_points_w,1),1)]';
            x_Rs = points_Rs(1,:)';
            y_Rs = points_Rs(2,:)';
            z_Rs = points_Rs(3,:)';

            % Get normalized coordinates
            x_Rn = x_Rs./z_Rs;
            y_Rn = y_Rs./z_Rs;
            r_Rn = sqrt(x_Rn.^2 + y_Rn.^2);           
            
            % Rows of jacobian and residuals
            x_top_R = y_bottom_L+1;
            x_bottom_R = x_top_R+num_points-1;
            y_top_R = x_bottom_R+1;
            y_bottom_R = y_top_R+num_points-1;
            
            % Intrinsic params
            % x coords
            jacob(x_top_R:x_bottom_R,9) = x_Rn.*(1+beta_R1*r_Rn.^2+beta_R2*r_Rn.^4) + 2*beta_R3*x_Rn.*y_Rn + beta_R4*(r_Rn.^2 + 2*x_Rn.^2); %#ok<*SPRIX>
            jacob(x_top_R:x_bottom_R,10) = 0;
            jacob(x_top_R:x_bottom_R,11) = 1;
            jacob(x_top_R:x_bottom_R,12) = 0;
            jacob(x_top_R:x_bottom_R,13) = alpha_Rx*x_Rn.*r_Rn.^2;
            jacob(x_top_R:x_bottom_R,14) = alpha_Rx*x_Rn.*r_Rn.^4;
            jacob(x_top_R:x_bottom_R,15) = 2*alpha_Rx.*x_Rn.*y_Rn;
            jacob(x_top_R:x_bottom_R,16) = alpha_Rx*(r_Rn.^2+2*x_Rn.^2);
            
            % y coords
            jacob(y_top_R:y_bottom_R,9) = 0;
            jacob(y_top_R:y_bottom_R,10) = y_Rn.*(1+beta_R1*r_Rn.^2+beta_R2*r_Rn.^4) + beta_R3*(r_Rn.^2 + 2*y_Rn.^2) + 2*beta_R4*x_Rn.*y_Rn;
            jacob(y_top_R:y_bottom_R,11) = 0;
            jacob(y_top_R:y_bottom_R,12) = 1;
            jacob(y_top_R:y_bottom_R,13) = alpha_Ry*y_Rn.*r_Rn.^2;
            jacob(y_top_R:y_bottom_R,14) = alpha_Ry*y_Rn.*r_Rn.^4;
            jacob(y_top_R:y_bottom_R,15) = alpha_Ry*(r_Rn.^2+2*y_Rn.^2);
            jacob(y_top_R:y_bottom_R,16) = 2*alpha_Ry*x_Rn.*y_Rn;
            
            % Extrinsic params
            dx_Rn_dRt_R = [X_s./z_Rs zeros(num_points,1) -(X_s.*x_Rs)./z_Rs.^2 Y_s./z_Rs zeros(num_points,1) -(Y_s.*x_Rs)./z_Rs.^2 1./z_Rs zeros(num_points,1) -x_Rs./z_Rs.^2];
            dy_Rn_dRt_R = [zeros(num_points,1) X_s./z_Rs -(X_s.*y_Rs)./z_Rs.^2 zeros(num_points,1) Y_s./z_Rs -(Y_s.*y_Rs)./z_Rs.^2 zeros(num_points,1) 1./z_Rs -y_Rs./z_Rs.^2];
             
            % Do dRt_R_dm_L first
            dRt_R_dRt_L = [R_s         zeros(3)    zeros(3);
                           zeros(3)    R_s         zeros(3);
                           zeros(3)    zeros(3)    R_s];
            dRt_L_dm_L = [0                                                                        -sin(theta_Ly)*cos(theta_Lz)                -cos(theta_Ly)*sin(theta_Lz)                                             0   0   0;
                          0                                                                        -sin(theta_Ly)*sin(theta_Lz)                 cos(theta_Ly)*cos(theta_Lz)                                             0   0   0;
                          0                                                                        -cos(theta_Ly)                               0                                                                       0   0   0;
                          sin(theta_Lx)*sin(theta_Lz)+cos(theta_Lx)*sin(theta_Ly)*cos(theta_Lz)     sin(theta_Lx)*cos(theta_Ly)*cos(theta_Lz)  -cos(theta_Lx)*cos(theta_Lz)-sin(theta_Lx)*sin(theta_Ly)*sin(theta_Lz)   0   0   0;
                         -sin(theta_Lx)*cos(theta_Lz)+cos(theta_Lx)*sin(theta_Ly)*sin(theta_Lz)     sin(theta_Lx)*cos(theta_Ly)*sin(theta_Lz)  -cos(theta_Lx)*sin(theta_Lz)+sin(theta_Lx)*sin(theta_Ly)*cos(theta_Lz)   0   0   0;
                          cos(theta_Lx)*cos(theta_Ly)                                              -sin(theta_Lx)*sin(theta_Ly)                 0                                                                       0   0   0;
                          0                                                                         0                                           0                                                                       1   0   0;
                          0                                                                         0                                           0                                                                       0   1   0;
                          0                                                                         0                                           0                                                                       0   0   1];       
            dRt_R_dm_L = dRt_R_dRt_L*dRt_L_dm_L;
            
            % Do dRt_R_dm_s next
            dRt_R_dRt_s = [R_L(1,1)*eye(3) R_L(2,1)*eye(3) R_L(3,1)*eye(3) zeros(3);
                           R_L(1,2)*eye(3) R_L(2,2)*eye(3) R_L(3,2)*eye(3) zeros(3);
                           t_L(1)*eye(3)   t_L(2)*eye(3)   t_L(3)*eye(3)   eye(3)];         
            dRt_s_dm_s = [0                                                                        -sin(theta_sy)*cos(theta_sz)                -cos(theta_sy)*sin(theta_sz)                                             0   0   0;
                          0                                                                        -sin(theta_sy)*sin(theta_sz)                 cos(theta_sy)*cos(theta_sz)                                             0   0   0;
                          0                                                                        -cos(theta_sy)                               0                                                                       0   0   0;
                          sin(theta_sx)*sin(theta_sz)+cos(theta_sx)*sin(theta_sy)*cos(theta_sz)     sin(theta_sx)*cos(theta_sy)*cos(theta_sz)  -cos(theta_sx)*cos(theta_sz)-sin(theta_sx)*sin(theta_sy)*sin(theta_sz)   0   0   0;
                         -sin(theta_sx)*cos(theta_sz)+cos(theta_sx)*sin(theta_sy)*sin(theta_sz)     sin(theta_sx)*cos(theta_sy)*sin(theta_sz)  -cos(theta_sx)*sin(theta_sz)+sin(theta_sx)*sin(theta_sy)*cos(theta_sz)   0   0   0;
                          cos(theta_sx)*cos(theta_sy)                                              -sin(theta_sx)*sin(theta_sy)                 0                                                                       0   0   0;
                          cos(theta_sx)*sin(theta_sz)-sin(theta_sx)*sin(theta_sy)*cos(theta_sz)     cos(theta_sx)*cos(theta_sy)*cos(theta_sz)   sin(theta_sx)*cos(theta_sz)-cos(theta_sx)*sin(theta_sy)*sin(theta_sz)   0   0   0;
                         -cos(theta_sx)*cos(theta_sz)-sin(theta_sx)*sin(theta_sy)*sin(theta_sz)     cos(theta_sx)*cos(theta_sy)*sin(theta_sz)   sin(theta_sx)*sin(theta_sz)+cos(theta_sx)*sin(theta_sy)*cos(theta_sz)   0   0   0;
                         -sin(theta_sx)*cos(theta_sy)                                              -cos(theta_sx)*sin(theta_sy)                 0                                                                       0   0   0;
                          0                                                                         0                                           0                                                                       1   0   0;
                          0                                                                         0                                           0                                                                       0   1   0;
                          0                                                                         0                                           0                                                                       0   0   1];
            dRt_R_dm_s = dRt_R_dRt_s*dRt_s_dm_s;
                      
            % Get jacobian of normalized coords
            dx_Rn_dm_L = dx_Rn_dRt_R*dRt_R_dm_L;
            dy_Rn_dm_L = dy_Rn_dRt_R*dRt_R_dm_L;
            dr_Rn_dm_L = (x_Rn.*dx_Rn_dm_L + y_Rn.*dy_Rn_dm_L)./r_Rn;
            
            % Store jacobian of pixel coords
            jacob(x_top_R:x_bottom_R,16+(i-1)*6+1:16+i*6) = alpha_Rx*(dx_Rn_dm_L.*(1+beta_R1*r_Rn.^2+beta_R2*r_Rn.^4)+x_Rn.*(2*beta_R1*r_Rn.*dr_Rn_dm_L+4*beta_R2*r_Rn.^3.*dr_Rn_dm_L)+2*beta_R3*(dx_Rn_dm_L.*y_Rn+x_Rn.*dy_Rn_dm_L)+beta_R4*(2*r_Rn.*dr_Rn_dm_L+4*x_Rn.*dx_Rn_dm_L));           
            jacob(y_top_R:y_bottom_R,16+(i-1)*6+1:16+i*6) = alpha_Ry*(dy_Rn_dm_L.*(1+beta_R1*r_Rn.^2+beta_R2*r_Rn.^4)+y_Rn.*(2*beta_R1*r_Rn.*dr_Rn_dm_L+4*beta_R2*r_Rn.^3.*dr_Rn_dm_L)+beta_R3*(2*r_Rn.*dr_Rn_dm_L+4*y_Rn.*dy_Rn_dm_L)+2*beta_R4*(dx_Rn_dm_L.*y_Rn+x_Rn.*dy_Rn_dm_L)); 
                        
            % Get jacobian of normalized coords
            dx_Rn_dm_s = dx_Rn_dRt_R*dRt_R_dm_s;
            dy_Rn_dm_s = dy_Rn_dRt_R*dRt_R_dm_s;
            dr_Rn_dm_s = (x_Rn.*dx_Rn_dm_s + y_Rn.*dy_Rn_dm_s)./r_Rn;
            
            % Store jacobian of pixel coords
            jacob(x_top_R:x_bottom_R,16+num_boards*6+1:16+(num_boards+1)*6) = alpha_Rx*(dx_Rn_dm_s.*(1+beta_R1*r_Rn.^2+beta_R2*r_Rn.^4)+x_Rn.*(2*beta_R1*r_Rn.*dr_Rn_dm_s+4*beta_R2*r_Rn.^3.*dr_Rn_dm_s)+2*beta_R3*(dx_Rn_dm_s.*y_Rn+x_Rn.*dy_Rn_dm_s)+beta_R4*(2*r_Rn.*dr_Rn_dm_s+4*x_Rn.*dx_Rn_dm_s));           
            jacob(y_top_R:y_bottom_R,16+num_boards*6+1:16+(num_boards+1)*6) = alpha_Ry*(dy_Rn_dm_s.*(1+beta_R1*r_Rn.^2+beta_R2*r_Rn.^4)+y_Rn.*(2*beta_R1*r_Rn.*dr_Rn_dm_s+4*beta_R2*r_Rn.^3.*dr_Rn_dm_s)+beta_R3*(2*r_Rn.*dr_Rn_dm_s+4*y_Rn.*dy_Rn_dm_s)+2*beta_R4*(dx_Rn_dm_s.*y_Rn+x_Rn.*dy_Rn_dm_s)); 
                                   
            % Compute projected points from model
            x_model_R = alpha_Rx*(x_Rn.*(1+beta_R1*r_Rn.^2+beta_R2*r_Rn.^4) + 2*beta_R3*x_Rn.*y_Rn + beta_R4*(r_Rn.^2+2*x_Rn.^2)) + x_Ro;
            y_model_R = alpha_Ry*(y_Rn.*(1+beta_R1*r_Rn.^2+beta_R2*r_Rn.^4) + beta_R3*(r_Rn.^2+2*y_Rn.^2) + 2*beta_R4*x_Rn.*y_Rn) + y_Ro;     
            
            % Store residuals
            board_points_Ri = board_points_is.R{i};
            res(x_top_R:x_bottom_R) = x_model_R - board_points_Ri(:,1);
            res(y_top_R:y_bottom_R) = y_model_R - board_points_Ri(:,2);   
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
        if norm(delta_p) < cb_config.refine_param_norm_cutoff
            break
        end
    end    
    if it == cb_config.refine_param_it_cutoff
        disp('WARNING: iterations hit cutoff before converging!!!');
    end
        
    % Get outputs from p    
    R_s = alg.euler2rot(p(16 +6*num_boards+1:16 +6*num_boards+3));
    t_s = p(16 +6*num_boards+4:16 +6*num_boards+6);
    rotations.L = {};
    rotations.R = {};
    translations.L = {};
    translations.R = {};
    for i = 1:num_boards
        rotations.L{i} = alg.euler2rot(p(16 +6*(i-1)+1:16 +6*(i-1)+3));
        translations.L{i} = p(16 +6*(i-1)+4:16 +6*(i-1)+6);  
        rotations.R{i} = R_s*rotations.L{i};  
        translations.R{i} = R_s*translations.L{i} + t_s;
    end
    A.L = [p(1)     0       p(3);
           0        p(2)    p(4);
           0        0       1];
    A.R = [p(9)     0       p(11);
           0        p(10)   p(12);
           0        0       1];
    distortion.L = [p(5); p(6); p(7); p(8)];
    distortion.R = [p(13); p(14); p(15); p(16)];
end
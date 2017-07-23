function [A,distortion,rotations,translations] = nonlinear_params(A,rotations,translations,board_points_is,cb_config)
    % This will compute nonlinear refinement given input A, rotations, and
    % translations. This takes into account the full camera model,
    % including radial and tangential distortions (with initial guess of 0
    % for these four parameters).
    %
    % Inputs:
    %   A - array; 3x3 array containing:
    %       [alpha_x    0       x_o;
    %        0          alpha_y y_o;
    %        0          0       1]
    %   rotations - cell; Mx1 cell array containing 3x3 rotation matrices
    %   translations - cell; Mx1 cell array containing 3x1 translation
    %       vectors
    %   board_points_is - cell; Mx1 cell array of calibration board points    
    %   cb_config - struct; this is the struct returned by
    %       util.load_cb_config()
    %
    % Outputs:
    %   A - array; optimized A
    %   distortion - array; 4x1 array ofoptimized distortions (radial and 
    %   tangential) stored as: 
    %       [beta_1; beta_2; beta_3; beta_4]
    %   rotations - cell; optimized rotations
    %   translations - cell; optimized translations
          
    % TODO: make sure rotations and translations have the same length
        
    % Get number of calibration boards and number of board points
    board_points_w = alg.cb_points(cb_config);
    X_s = board_points_w(:,1);
    Y_s = board_points_w(:,2);
    
    % Get number of boards and points
    num_boards = length(board_points_is);
    num_points = size(board_points_w,1);
    
    % Supply initial parameter vector. p has a length of 8 + 6*M, where M 
    % is the number of calibration boards. There are 8 intrinsic parameters
    % (4 more for distortion parameters) and 6 extrinsic parameters per 
    % board.
    % p has form of: 
    %   [alpha_x, alpha_y, x_o, y_o, beta_1, beta_2, beta_3, beta_4, ...
    %    theta_x1, theta_y1, theta_z1, t_x1, t_y1, t_z1, ... 
    %    theta_xM, theta_yM, theta_zM, t_xM, t_yM, t_zM]
    p = zeros(8+6*num_boards,1);
    
    % Do extrinsic parameters first
    p(1) = A(1,1);
    p(2) = A(2,2);
    p(3) = A(1,3);
    p(4) = A(2,3);
    % Let initial distortion params be zero
    
    % Cycle over rotations and translations and store params. 
    for i = 1:num_boards
        R = rotations{i};
        t = translations{i};
        euler = alg.rot2euler(R);        
        p(8 +6*(i-1)+1:8 +6*(i-1)+3) = euler;
        p(8 +6*(i-1)+4:8 +6*(i-1)+6) = t';        
    end
   
    % Initialize jacobian
    jacob = sparse(2*num_boards*num_points,8+6*num_boards);
    res = zeros(2*num_boards*num_points,1);
    
    % Perform gauss newton iterations until convergence
    it_cutoff = 20;
    norm_cutoff = 1e-5;
    for it = 1:it_cutoff      
        % Get parameters
        alpha_x = p(1);
        alpha_y = p(2);
        x_o = p(3);
        y_o = p(4);        
        beta_1 = p(5);
        beta_2 = p(6);
        beta_3 = p(7);
        beta_4 = p(8);
                    
        % Fill jacobian and residuals per board
        for i = 1:num_boards
            % Get rotation and translation for this board
            t = p(8+6*(i-1)+4:8+6*(i-1)+6);
            euler = p(8+6*(i-1)+1:8+6*(i-1)+3);
            R = alg.euler2rot(euler);
            theta_x = euler(1);
            theta_y = euler(2);
            theta_z = euler(3);

            % Apply xform to points to get into camera coordinates
            points_s = [R(:,1) R(:,2) t] * [board_points_w ones(size(board_points_w,1),1)]';
            x_s = points_s(1,:)';
            y_s = points_s(2,:)';
            z_s = points_s(3,:)';

            % Get normalized coordinates
            x_n = x_s./z_s;
            y_n = y_s./z_s;
            r_n = sqrt(x_n.^2 + y_n.^2);           
            
            % Rows of jacobian and residuals
            x_top = (i-1)*2*num_points+1;
            x_bottom = x_top+num_points-1;
            y_top = x_bottom+1;
            y_bottom = y_top+num_points-1;
            
            % Intrinsic params
            % x coords
            jacob(x_top:x_bottom,1) = x_n.*(1+beta_1*r_n.^2+beta_2*r_n.^4) + 2*beta_3*x_n.*y_n + beta_4*(r_n.^2 + 2*x_n.^2); %#ok<*SPRIX>
            jacob(x_top:x_bottom,2) = 0;
            jacob(x_top:x_bottom,3) = 1;
            jacob(x_top:x_bottom,4) = 0;
            jacob(x_top:x_bottom,5) = alpha_x*x_n.*r_n.^2;
            jacob(x_top:x_bottom,6) = alpha_x*x_n.*r_n.^4;
            jacob(x_top:x_bottom,7) = 2*alpha_x.*x_n.*y_n;
            jacob(x_top:x_bottom,8) = alpha_x*(r_n.^2+2*x_n.^2);
            
            % y coords
            jacob(y_top:y_bottom,1) = 0;
            jacob(y_top:y_bottom,2) = y_n.*(1+beta_1*r_n.^2+beta_2*r_n.^4) + beta_3*(r_n.^2 + 2*y_n.^2) + 2*beta_4*x_n.*y_n;
            jacob(y_top:y_bottom,3) = 0;
            jacob(y_top:y_bottom,4) = 1;
            jacob(y_top:y_bottom,5) = alpha_y*y_n.*r_n.^2;
            jacob(y_top:y_bottom,6) = alpha_y*y_n.*r_n.^4;
            jacob(y_top:y_bottom,7) = alpha_y*(r_n.^2+2*y_n.^2);
            jacob(y_top:y_bottom,8) = 2*alpha_y*x_n.*y_n;
            
            % Extrinsic params
            % First compute:
            %   dx_n/dtheta_x1, dx_n/dtheta_y1, dx_n/dtheta_z1, dx_n/dt_x, dx_n/dt_y, dx_n/dt_z
            %   dy_n/dtheta_x1, dy_n/dtheta_y1, dy_n/dtheta_z1, dy_n/dt_x, dy_n/dt_y, dy_n/dt_z
            dx_n_dRt = [X_s./z_s zeros(num_points,1) -(X_s.*x_s)./z_s.^2 Y_s./z_s zeros(num_points,1) -(Y_s.*x_s)./z_s.^2 1./z_s zeros(num_points,1) -x_s./z_s.^2];
            dy_n_dRt = [zeros(num_points,1) X_s./z_s -(X_s.*y_s)./z_s.^2 zeros(num_points,1) Y_s./z_s -(Y_s.*y_s)./z_s.^2 zeros(num_points,1) 1./z_s -y_s./z_s.^2];
             
            % m = [theta_x theta_y theta_z tx ty tz]
            dRt_dm = [0                                                                -sin(theta_y)*cos(theta_z)              -cos(theta_y)*sin(theta_z)                                           0   0   0;
                      0                                                                -sin(theta_y)*sin(theta_z)               cos(theta_y)*cos(theta_z)                                           0   0   0;
                      0                                                                -cos(theta_y)                            0                                                                   0   0   0;
                      sin(theta_x)*sin(theta_z)+cos(theta_x)*sin(theta_y)*cos(theta_z)  sin(theta_x)*cos(theta_y)*cos(theta_z) -cos(theta_x)*cos(theta_z)-sin(theta_x)*sin(theta_y)*sin(theta_z)    0   0   0;
                     -sin(theta_x)*cos(theta_z)+cos(theta_x)*sin(theta_y)*sin(theta_z)  sin(theta_x)*cos(theta_y)*sin(theta_z) -cos(theta_x)*sin(theta_z)+sin(theta_x)*sin(theta_y)*cos(theta_z)    0   0   0;
                      cos(theta_x)*cos(theta_y)                                        -sin(theta_x)*sin(theta_y)               0                                                                   0   0   0;
                      0                                                                 0                                       0                                                                   1   0   0;
                      0                                                                 0                                       0                                                                   0   1   0;
                      0                                                                 0                                       0                                                                   0   0   1];
            
            % Get jacobian of normalized coords
            dx_n_dm = dx_n_dRt*dRt_dm;
            dy_n_dm = dy_n_dRt*dRt_dm;
            dr_n_dm = (x_n.*dx_n_dm + y_n.*dy_n_dm)./r_n;
            
            % Store jacobian of pixel coords
            jacob(x_top:x_bottom,8+(i-1)*6+1:8+i*6) = alpha_x*(dx_n_dm.*(1+beta_1*r_n.^2+beta_2*r_n.^4)+x_n.*(2*beta_1*r_n.*dr_n_dm+4*beta_2*r_n.^3.*dr_n_dm)+2*beta_3*(dx_n_dm.*y_n+x_n.*dy_n_dm)+beta_4*(2*r_n.*dr_n_dm+4*x_n.*dx_n_dm));           
            jacob(y_top:y_bottom,8+(i-1)*6+1:8+i*6) = alpha_y*(dy_n_dm.*(1+beta_1*r_n.^2+beta_2*r_n.^4)+y_n.*(2*beta_1*r_n.*dr_n_dm+4*beta_2*r_n.^3.*dr_n_dm)+beta_3*(2*r_n.*dr_n_dm+4*y_n.*dy_n_dm)+2*beta_4*(dx_n_dm.*y_n+x_n.*dy_n_dm)); 
            
            % Compute residuals        
            board_points_i = board_points_is{i};
            
            % Compute projected points from model
            x_model = alpha_x*(x_n.*(1+beta_1*r_n.^2+beta_2*r_n.^4) + 2*beta_3*x_n.*y_n + beta_4*(r_n.^2+2*x_n.^2)) + x_o;
            y_model = alpha_y*(y_n.*(1+beta_1*r_n.^2+beta_2*r_n.^4) + beta_3*(r_n.^2+2*y_n.^2) + 2*beta_4*x_n.*y_n) + y_o;     
            
            % Store residuals
            res(x_top:x_bottom) = x_model - board_points_i(:,1);
            res(y_top:y_bottom) = y_model - board_points_i(:,2);           
        end  
        
        % Get and store update
        delta_p = -inv(jacob'*jacob)*jacob'*res;
        p = p + delta_p;
        
        % Exit if change in distance is small
        norm_delta_p = norm(delta_p);        
        disp(['Iteration #: ' num2str(it)]);
        disp(['Difference norm for nonlinear parameter refinement: ' num2str(norm_delta_p)]);
        if norm(delta_p) < norm_cutoff
            break
        end
    end    
    if it == it_cutoff
        disp('WARNING: iterations hit cutoff before converging!!!');
    end
        
    % Get A, rotations, and distortions from p
    A = [p(1) 0    p(3);
         0    p(2) p(4);
         0    0    1];

    distortion = [p(5); p(6); p(7); p(8)];
    
    translations = {};
    rotations = {};
    for i = 1:num_boards
        % Get rotation and translation for this board
        translations{i} = p(8+6*(i-1)+4:8+6*(i-1)+6); %#ok<AGROW>
        rotations{i} = alg.euler2rot(p(8+6*(i-1)+1:8+6*(i-1)+3)); %#ok<AGROW>
    end
end
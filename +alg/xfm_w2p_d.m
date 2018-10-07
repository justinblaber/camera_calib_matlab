function p_p_d_ms = xfm_w2p_d(A,d,R,t,f_p_p_d,p_ws,opts)
    
    % Validate inputs
    util.validate_A(A);
    
    % Get camera matrix components
    alpha = A(1,1);
    x_o = A(1,3);
    y_o = A(2,3);   
    
    % Convert d to cell - used to pass distortion arguments into function
    % handle
    d_cell = num2cell(d);
    
    % Form homography
    H = A*[R(:,1) R(:,2) t];
    
    % Apply it    
    switch opts.target_type
        case 'checker'
            % Use "point to point" homography estimation
            p_ps = alg.apply_homography_p2p(H,p_ws);
        case 'circle'
            % Use "circle to ellipse" homography estimation
            p_ps = alg.apply_homography_c2e(H,p_ws,opts.circle_radius);  
        otherwise
            error(['Unknown target type: "' opts.target_type '"']);
    end 
    
    % Apply distortion
    x_p_bars = p_ps(:,1) - x_o;
    y_p_bars = p_ps(:,2) - y_o;
    p_p_d_ms = f_p_p_d(x_p_bars,y_p_bars,alpha,x_o,y_o,d_cell{:});
end

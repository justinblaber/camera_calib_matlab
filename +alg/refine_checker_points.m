function [board_points_p, idx_valid, cov, debug] = refine_checker_points(array,forward_xfm,opts)
    % Performs refinement of checker points on an array.
    %
    % Inputs:
    %   array - array; MxN array
    %   forward_xfm - function handle; function which transforms world
    %   	coordinates to pixel coordinates
    %   opts - struct;
    %       .four_point_height - scalar; height of the "four point" box
    %       .four_point_width - scalar; width of the "four point" box
    %       .num_targets_height - int; number of targets in the "height" 
    %           dimension
    %       .num_targets_width - int; number of targets in the "width"
    %           dimension
    %       .target_spacing - scalar; space between targets
    %       .refine_checker_max_hw - int; max half window for checker
    %           refinement
    %       .refine_checker_opencv_it_cutoff - int; max number of 
    %           iterations performed for "opencv" checker refinement
    %       .refine_checker_opencv_norm_cutoff - scalar; cutoff for the 
    %           difference in norm of center position for "opencv" checker
    %           refinement
    %       .refine_checker_edges_h1_init - scalar; initial value of h1
    %           parameter in "edges" checker refinement
    %       .refine_checker_edges_h2_init -  scalar; initial value of h2
    %           parameter in "edges" checker refinement
    %       .refine_checker_edges_it_cutoff - int; max number of 
    %           iterations performed for "edges" checker refinement 
    %       .refine_checker_edges_norm_cutoff  - scalar; cutoff for the 
    %           difference in norm of the parameter vector for "edges" 
    %           checker refinement
    %
    % Outputs:
    %   board_points_p - array; Px2 array of optimized subpixel checker
    %       points in pixel coordinates.
    %   idx_valid - array; Px1 logical array of "valid" checker points
    %   cov - cell array; Px1 cell array of covariance matrices for
    %       detected corners
    %   debug - cell array; Px1 cell array of bounding boxes of the
    %       refinement windows.
    
    % Get board points in world coordinates
    board_points_w = alg.cb_points(opts);
    
    % Get interpolator
    I_array = griddedInterpolant({1:size(array,1),1:size(array,2)}, ...
                                 array,'linear','none');

    % Cycle over points and refine them; also keep track of which indices
    % are "valid"
    board_points_p = zeros(size(board_points_w));
    idx_valid = false(size(board_points_w,1),1);
    cov = cell(size(board_points_w,1),1);
    for i = 1:size(board_points_w,1)
        % Get point
        p_w = board_points_w(i,:);
        
        % Get box around point
        % Note:
        %    p1 - l2 - p3
        %    |         |
        %    l1   p    l4
        %    |         |
        %    p2 - l3 - p4
        box_w = [p_w(1)-opts.target_spacing p_w(2)-opts.target_spacing;
                 p_w(1)-opts.target_spacing p_w(2)+opts.target_spacing;
                 p_w(1)+opts.target_spacing p_w(2)-opts.target_spacing;
                 p_w(1)+opts.target_spacing p_w(2)+opts.target_spacing];
             
        % Apply xform to go from world coordinates to pixel coordinates
        box_p = forward_xfm(box_w);
        
        % Form lines
        l1_p = alg.points2line(box_p(1,:),box_p(2,:));
        l2_p = alg.points2line(box_p(1,:),box_p(3,:));
        l3_p = alg.points2line(box_p(2,:),box_p(4,:));
        l4_p = alg.points2line(box_p(3,:),box_p(4,:));
        
        % Get shortest distance from lines to target point in pixel
        % coordinates
        p_p = forward_xfm(p_w);        
        d1 = alg.point_line_distance(p_p,l1_p);
        d2 = alg.point_line_distance(p_p,l2_p);
        d3 = alg.point_line_distance(p_p,l3_p);
        d4 = alg.point_line_distance(p_p,l4_p);
        
        % Get minimum distance
        d_min = min([d1 d2 d3 d4]);
        
        % Get half width of circumscribed square, which should help prevent
        % neighboring checkers from being inside the square.
        hw_sub_array = floor(d_min/sqrt(2));
        if hw_sub_array < opts.refine_checker_min_hw
            hw_sub_array = opts.refine_checker_min_hw;
            warning(['Minimum half width set for point: ' num2str(i) '!!!']);
        elseif hw_sub_array > opts.refine_checker_max_hw
            hw_sub_array = opts.refine_checker_max_hw;
        end
        
        % Perform initial refinement with "opencv" checker detection. This
        % method is pretty robust and reasonably accurate.
        diff_norm = Inf; % Initialize in case loop is exited before first iteration
        for it = 1:opts.refine_checker_opencv_it_cutoff            
            % Get coordinates of sub array
            [y,x] = ndgrid(p_p(2)-hw_sub_array:p_p(2)+hw_sub_array, ...
                           p_p(1)-hw_sub_array:p_p(1)+hw_sub_array);
            x = x(:);
            y = y(:);
                       
            % Check bounds; if broken here, this index will remain as
            % being invalid
            if any(x < 1 | x > size(array,2)) || ...
               any(y < 1 | y > size(array,1))
                break
            end
            
            % Get sub array
            sub_array = reshape(I_array(y,x), ...
                                [2*hw_sub_array+1 2*hw_sub_array+1]);
            
            % Cache previous point
            p_prev = p_p;
                           
            % Get refined point; note that coordinates will be WRT
            % sub_array.
            p_p_sub = alg.refine_checker_opencv(sub_array);
            
            % Get point in array coordinates.
            p_p = p_p_sub + (p_p-hw_sub_array-1);           
                                  
            % Exit if change in distance is small
            diff_norm = norm(p_prev-p_p);
            if diff_norm < opts.refine_checker_opencv_norm_cutoff
                break
            end
        end
                        
        % "opencv" refinement is done; if successful; perform a final
        % "edges" refinement.
        if diff_norm < opts.refine_checker_opencv_norm_cutoff || it == opts.refine_checker_opencv_it_cutoff           
            % Get coordinates of sub array
            [y,x] = ndgrid(p_p(2)-hw_sub_array:p_p(2)+hw_sub_array, ...
                           p_p(1)-hw_sub_array:p_p(1)+hw_sub_array);
            x = x(:);
            y = y(:);
                       
            % Check bounds; if broken here, this index will remain as
            % being invalid
            if any(x < 1 | x > size(array,2)) || ...
               any(y < 1 | y > size(array,1))
                break
            end
            
            % Get sub array
            sub_array = reshape(I_array(y,x), ...
                                [2*hw_sub_array+1 2*hw_sub_array+1]);
            
            % Get initial guesses for two lines in "edges" method.
            % Note:
            %         p2
            %         |
            %   p1 -  p - p4
            %         |
            %         p3
            p1_p = forward_xfm([p_w(1)-opts.target_spacing p_w(2)]);
            p2_p = forward_xfm([p_w(1) p_w(2)-opts.target_spacing]);
            p3_p = forward_xfm([p_w(1) p_w(2)+opts.target_spacing]);
            p4_p = forward_xfm([p_w(1)+opts.target_spacing p_w(2)]);

            % Get points in sub array coordinates
            p1_p_sub = p1_p - (p_p-hw_sub_array-1);
            p2_p_sub = p2_p - (p_p-hw_sub_array-1);
            p3_p_sub = p3_p - (p_p-hw_sub_array-1);
            p4_p_sub = p4_p - (p_p-hw_sub_array-1);

            % Get lines initial line estimates
            l1_p = alg.pointslope2line([hw_sub_array+1 hw_sub_array+1], ...
                                       (p4_p_sub(2)-p1_p_sub(2))/(p4_p_sub(1)-p1_p_sub(1)));
            l2_p = alg.pointslope2line([hw_sub_array+1 hw_sub_array+1], ...
                                       (p3_p_sub(2)-p2_p_sub(2))/(p3_p_sub(1)-p2_p_sub(1)));
                                       
            % Get refined point; note that coordinates will be WRT
            % sub_array.
            [p_p_sub, cov{i}] = alg.refine_checker_edges(sub_array, ...
                                                         l1_p, ...
                                                         l2_p, ...
                                                         opts);
                        
            % Get point in array coordinates.
            p_p = p_p_sub + (p_p-hw_sub_array-1);     
            
            % Store point, mark as valid, and store bounding box of sub
            % array for debugging purposes
            board_points_p(i,:) = p_p;
            idx_valid(i) = true;
            debug{i} = [min(x) min(y); ...
                        max(x) max(y)]; %#ok<AGROW>
        end
    end    
end
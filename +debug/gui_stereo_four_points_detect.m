function gui_stereo_four_points_detect(four_points_ps,four_points_debugs,cb_imgs,calib_config,f)
    % GUI for debugging four point detection
            
    if ~exist('f','var')
        f = figure(); 
    end
    set(f,'Interruptible','off');
    
    % Disable KeyPressFcn until after plotting is complete
    set(f,'KeyPressFcn',@(~,~)drawnow);
                
    % Initialize parameters
    mode = 'whole';
    idx_board = 1;
    num_boards = length(cb_imgs.L); 
    axes_cal_board_L = matlab.graphics.axis.Axes.empty();
    axes_cal_board_R = matlab.graphics.axis.Axes.empty();
    
    % Set axes parameters
    padding_height = 0.075;
    padding_width = 0.025;
    cal_board_width = 0.35;
    
    % Initialize plot
    plot_gui();
    
    % Set bounds
    set_bounds();
    
    % Set KeyPressFcn callback
    set(f,'KeyPressFcn',@KeyPressFcn);
    
    function KeyPressFcn(~,eventData)   
        try        
            % Disable KeyPressFcn until after this is done
            set(f,'KeyPressFcn',@(~,~)drawnow);
            
            % Set idx_board
            replot = false;
            switch eventData.Key
                case 'rightarrow'
                    if idx_board < num_boards
                        idx_board = idx_board+1;
                        replot = true;
                    else
                        % Set KeyPressFcn callback
                        set(f,'KeyPressFcn',@KeyPressFcn);  
                        return
                    end
                case 'leftarrow'
                    if idx_board > 1
                        idx_board = idx_board-1;
                        replot = true;
                    else
                        % Set KeyPressFcn callback
                        set(f,'KeyPressFcn',@KeyPressFcn);  
                        return
                    end
                case 'escape'
                    mode = 'whole';
                case '1'
                    if strcmp(mode,'1')
                        mode = 'whole';
                    else
                        mode = '1';
                    end
                case '2'
                    if strcmp(mode,'2')
                        mode = 'whole';
                    else
                        mode = '2';
                    end
                case '3'
                    if strcmp(mode,'3')
                        mode = 'whole';
                    else
                        mode = '3';
                    end
                case '4'
                    if strcmp(mode,'4')
                        mode = 'whole';
                    else
                        mode = '4';
                    end
                case 'w'
                    if strcmp(mode,'worst')
                        mode = 'whole';
                    else
                        mode = 'worst';
                    end
                otherwise
                    % Set KeyPressFcn callback
                    set(f,'KeyPressFcn',@KeyPressFcn);  
                    return
            end
            
            % Replot
            if replot
                plot_gui();   
            end
            
            % Set bounds
            set_bounds();

            % Set KeyPressFcn callback
            set(f,'KeyPressFcn',@KeyPressFcn);  
        catch e        
            if ishandle(f)
                rethrow(e);
            end
        end
    end

    function plot_gui()  
        try        
            % Clear figure and replot everything for simplicity
            clf(f);

            % Set axes  
            single_patch_height = (1-5*padding_height)/4;
            single_patch_width = (1-2*cal_board_width-5*padding_width)/2;
            
            pos_cal_board_L = [2*padding_width+single_patch_width padding_height cal_board_width 1-2*padding_height];
            axes_cal_board_L = axes('Position',pos_cal_board_L,'Parent',f);  
            pos_cal_board_R = [pos_cal_board_L(1)+pos_cal_board_L(3)+padding_width pos_cal_board_L(2:4)];
            axes_cal_board_R = axes('Position',pos_cal_board_R,'Parent',f);  

            axes_patches_L = matlab.graphics.axis.Axes.empty();
            axes_patches_R = matlab.graphics.axis.Axes.empty();
            for i = 1:4
                pos_patch_L = [padding_width ...
                               padding_height+(4-i)*(single_patch_height+padding_height)  ...
                               single_patch_width ...
                               single_patch_height];
                axes_patches_L(i) = axes('Position',pos_patch_L,'Parent',f);

                pos_patch_R = [pos_cal_board_R(1)+pos_cal_board_R(3)+padding_width ...
                               padding_height+(4-i)*(single_patch_height+padding_height)  ...
                               single_patch_width ...
                               single_patch_height];
                axes_patches_R(i) = axes('Position',pos_patch_R,'Parent',f);
            end

            % Plot debugging info
            debug.plot_four_point_debug(four_points_ps.L{idx_board}, ...
                                        four_points_debugs.L(idx_board), ...
                                        cb_imgs.L(idx_board), ...
                                        calib_config, ...
                                        axes_cal_board_L);  
            title(axes_cal_board_L,'Blobs, ellipses, and four points (L)','FontSize',10); 
            xlabel(axes_cal_board_L,['Path: ' cb_imgs.L(idx_board).get_path()], ...
                   'FontSize',8,'Interpreter','none');    
               
            debug.plot_four_point_debug(four_points_ps.R{idx_board}, ...
                                        four_points_debugs.R(idx_board), ...
                                        cb_imgs.R(idx_board), ...
                                        calib_config, ...
                                        axes_cal_board_R);  
            title(axes_cal_board_R,'Blobs, ellipses, and four points (R)','FontSize',10); 
            xlabel(axes_cal_board_R,['Path: ' cb_imgs.R(idx_board).get_path()], ...
                   'FontSize',8,'Interpreter','none');               
            
            % Plot patches
            for i = 1:4
                debug.plot_patch(four_points_debugs.L(idx_board).patch_matches(i).patch, ...
                                 four_points_debugs.L(idx_board).patch_matches(i).template, ...
                                 i, ...
                                 four_points_debugs.L(idx_board).patch_matches(i).cc_val, ...
                                 axes_patches_L(i));
            end
            for i = 1:4
                debug.plot_patch(four_points_debugs.R(idx_board).patch_matches(i).patch, ...
                                 four_points_debugs.R(idx_board).patch_matches(i).template, ...
                                 i, ...
                                 four_points_debugs.R(idx_board).patch_matches(i).cc_val, ...
                                 axes_patches_R(i));
            end
        catch e        
            if ishandle(f)
                rethrow(e);
            end
        end
    end

    function set_bounds()         
        try    
            % Set name
            set(f,'Name',['Board: ' num2str(idx_board) ' of ' num2str(num_boards) '; mode: ' mode '; (NOTE: press left, right, "1", "2", "3", "4", "w", and "esc" key arrows to toggle)']);
    
            % Set bounding box
            switch mode
                case 'whole'      
                    [l_L, r_L, t_L, b_L] = img_bb(cb_imgs.L(idx_board),calib_config);
                    [l_R, r_R, t_R, b_R] = img_bb(cb_imgs.R(idx_board),calib_config);
                case '1' 
                    [l_L, r_L, t_L, b_L] = ellipse_bb(four_points_debugs.L(idx_board).patch_matches(1).ellipse);
                    [l_R, r_R, t_R, b_R] = ellipse_bb(four_points_debugs.R(idx_board).patch_matches(1).ellipse);
                case '2'
                    [l_L, r_L, t_L, b_L] = ellipse_bb(four_points_debugs.L(idx_board).patch_matches(2).ellipse);
                    [l_R, r_R, t_R, b_R] = ellipse_bb(four_points_debugs.R(idx_board).patch_matches(2).ellipse);
                case '3'
                    [l_L, r_L, t_L, b_L] = ellipse_bb(four_points_debugs.L(idx_board).patch_matches(3).ellipse);
                    [l_R, r_R, t_R, b_R] = ellipse_bb(four_points_debugs.R(idx_board).patch_matches(3).ellipse);
                case '4'
                    [l_L, r_L, t_L, b_L] = ellipse_bb(four_points_debugs.L(idx_board).patch_matches(4).ellipse);
                    [l_R, r_R, t_R, b_R] = ellipse_bb(four_points_debugs.R(idx_board).patch_matches(4).ellipse);
                case 'worst'
                    % Get the worst patch
                    [~,idx_L] = min([four_points_debugs.L(idx_board).patch_matches.cc_val]);
                    [l_L, r_L, t_L, b_L] = ellipse_bb(four_points_debugs.L(idx_board).patch_matches(idx_L).ellipse);
                    [~,idx_R] = min([four_points_debugs.R(idx_board).patch_matches.cc_val]);
                    [l_R, r_R, t_R, b_R] = ellipse_bb(four_points_debugs.R(idx_board).patch_matches(idx_R).ellipse);
            end
            set(axes_cal_board_L, ...
                'Xlim',[l_L r_L], ...
                'Ylim',[t_L b_L]);
            set(axes_cal_board_R, ...
                'Xlim',[l_R r_R], ...
                'Ylim',[t_R b_R]);
        catch e        
            if ishandle(f)
                rethrow(e);
            end
        end
    end
end

function [l, r, t, b] = img_bb(cb_img,calib_config)
    img_height = cb_img.get_height();
    img_width = cb_img.get_width();
    if calib_config.four_point_detect_scaled_array_min_size == realmax
        scale_factor = 1;
    else
        scale_factor = calib_config.four_point_detect_scaled_array_min_size/min([img_height img_width]);
    end
    l = 0.5;
    r = img_width*scale_factor+0.5;
    t = 0.5;
    b = img_height*scale_factor+0.5;
end

function [l, r, t, b] = ellipse_bb(ellipse)
    zoom_factor = 10;
    width_ellipse = sqrt(ellipse.r1^2*cos(ellipse.rot)^2 + ellipse.r2^2*sin(ellipse.rot)^2);
    height_ellipse = sqrt(ellipse.r1^2*sin(ellipse.rot)^2 + ellipse.r2^2*cos(ellipse.rot)^2); 
    max_size = max(width_ellipse,height_ellipse);
    l = ellipse.x-zoom_factor*max_size;
    r = ellipse.x+zoom_factor*max_size;
    t = ellipse.y-zoom_factor*max_size;
    b = ellipse.y+zoom_factor*max_size;
end
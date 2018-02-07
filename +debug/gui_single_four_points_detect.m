function gui_single_four_points_detect(four_points_ps,four_points_debugs,cb_imgs,calib_config,f)
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
    num_boards = length(cb_imgs); 
    axes_cal_board = matlab.graphics.axis.Axes.empty();
    
    % Set axes parameters
    padding_height = 0.075;
    padding_width = 0.025;
    cal_board_width = 0.7;
    
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
            single_patch_width = 1-cal_board_width-3*padding_width;
            
            pos_cal_board = [1-cal_board_width-padding_width padding_height cal_board_width 1-2*padding_height];
            axes_cal_board = axes('Position',pos_cal_board,'Parent',f);  

            axes_patches = matlab.graphics.axis.Axes.empty();
            for i = 1:4
                pos_patch = [padding_width ...
                             padding_height+(4-i)*(single_patch_height+padding_height)  ...
                             single_patch_width ...
                             single_patch_height];
                axes_patches(i) = axes('Position',pos_patch,'Parent',f);
            end

            % Plot debugging info
            debug.plot_four_point_debug(four_points_ps{idx_board}, ...
                                        four_points_debugs(idx_board), ...
                                        cb_imgs(idx_board), ...
                                        calib_config, ...
                                        axes_cal_board);      
            title(axes_cal_board,'Blobs, ellipses, and four points','FontSize',10); 
            xlabel(axes_cal_board,['Path: ' cb_imgs(idx_board).get_path()], ...
                   'FontSize',8,'Interpreter','none');   
               
            % Plot patches
            for i = 1:4
                debug.plot_patch(four_points_debugs(idx_board).patch_matches(i).patch, ...
                                 four_points_debugs(idx_board).patch_matches(i).template, ...
                                 i, ...
                                 four_points_debugs(idx_board).patch_matches(i).cc_val, ...
                                 axes_patches(i));
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
                    [l, r, t, b] = img_bb(cb_imgs(idx_board),calib_config);
                case '1' 
                    [l, r, t, b] = ellipse_bb(four_points_debugs(idx_board).patch_matches(1).ellipse);
                case '2'
                    [l, r, t, b] = ellipse_bb(four_points_debugs(idx_board).patch_matches(2).ellipse);
                case '3'
                    [l, r, t, b] = ellipse_bb(four_points_debugs(idx_board).patch_matches(3).ellipse);
                case '4'
                    [l, r, t, b] = ellipse_bb(four_points_debugs(idx_board).patch_matches(4).ellipse);
                case 'worst'
                    % Get the worst patch
                    [~,idx] = min([four_points_debugs(idx_board).patch_matches.cc_val]);
                    [l, r, t, b] = ellipse_bb(four_points_debugs(idx_board).patch_matches(idx).ellipse);
            end
            set(axes_cal_board, ...
                'Xlim',[l r], ...
                'Ylim',[t b]);
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
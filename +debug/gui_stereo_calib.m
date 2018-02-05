function gui_stereo_calib(calib,R_s,t_s,f)
    % GUI for stereo calibration
            
    if ~exist('f','var')
        f = figure(); 
    end
    set(f,'Interruptible','off');
    
    % Disable KeyPressFcn until after plotting is complete
    set(f,'KeyPressFcn',@(~,~)drawnow);
                 
    % Initialize parameters
    mode = 'whole';
    idx_board = 1;
    num_boards = length(calib.L.extrin);                               
    alphas = 0.1*ones(1,num_boards);
    alphas(idx_board) = 1;             
    colors = external.distinguishable_colors(num_boards,{'w','r','k'});
    axes_board_L = matlab.graphics.axis.Axes.empty();
    axes_board_R = matlab.graphics.axis.Axes.empty();
    res_L = {};
    res_R = {};
    for i = 1:num_boards
        % Left
        board_points_m_L = alg.p_m(calib.L.intrin.A, ...
                                   calib.L.intrin.distortion, ...
                                   calib.L.extrin(i).rotation, ...
                                   calib.L.extrin(i).translation, ...
                                   alg.cb_points(calib.L.config));
        res_L{i} = board_points_m_L-calib.L.extrin(i).board_points_p; %#ok<AGROW>

        % Right
        board_points_m_R = alg.p_m(calib.R.intrin.A, ...
                                   calib.R.intrin.distortion, ...
                                   R_s*calib.L.extrin(i).rotation, ...
                                   R_s*calib.L.extrin(i).translation+t_s, ...
                                   alg.cb_points(calib.L.config));
        res_R{i} = board_points_m_R-calib.R.extrin(i).board_points_p; %#ok<AGROW>
    end 
    max_res = max([max(cellfun(@(x)max(abs(x(:))),res_L)) ...
                   max(cellfun(@(x)max(abs(x(:))),res_R))]);
    
    % Set axes parameters
    padding_height = 0.1;
    padding_width = 0.05;
    extrinsics_height = 0.25;
    extrinsics_width = 0.2;
    res_height = extrinsics_height;
    
    % Initialize plot
    plot_gui();
    set_bounds()
    
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

            % Set alphas
            alphas = 0.1*ones(1,num_boards);
            alphas(idx_board) = 1;             

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
            pos_extrinsics = [padding_width 1-padding_height-extrinsics_height extrinsics_width extrinsics_height];
            axes_extrinsics = axes('Position',pos_extrinsics,'Parent',f);

            pos_cal_board = [padding_width padding_height pos_extrinsics(3) pos_extrinsics(2)-2*padding_height];
            axes_cal_board = axes('Position',pos_cal_board,'Parent',f);        

            pos_res_L = [pos_cal_board(1)+pos_cal_board(3)+padding_width 1-padding_height-res_height (1-(pos_cal_board(1)+pos_cal_board(3))-3*padding_width)/2 res_height];
            axes_res_L = axes('Position',pos_res_L,'Parent',f);

            pos_res_R = [pos_res_L(1)+pos_res_L(3)+padding_width pos_res_L(2) pos_res_L(3) pos_res_L(4)];
            axes_res_R = axes('Position',pos_res_R,'Parent',f);

            pos_board_L = [pos_res_L(1) pos_cal_board(2) pos_res_L(3) pos_res_L(2)-2*padding_height];
            axes_board_L = axes('Position',pos_board_L,'Parent',f);

            pos_board_R = [pos_res_R(1) pos_cal_board(2) pos_res_R(3) pos_res_R(2)-2*padding_height];
            axes_board_R = axes('Position',pos_board_R,'Parent',f);

            % Plot
            rotations.L = {calib.L.extrin.rotation};
            rotations.R = {calib.R.extrin.rotation};
            translations.L = {calib.L.extrin.translation};
            translations.R = {calib.R.extrin.translation};
            debug.plot_stereo_extrinsic(rotations, ...
                                        translations, ...
                                        R_s, ...
                                        t_s, ...
                                        colors, ...
                                        alphas, ...
                                        calib.L.config, ...
                                        axes_extrinsics);  
            title(axes_extrinsics,'Extrinsics','FontSize',10); 
            drawnow
            
            debug.plot_cb_board_info_2D(calib.L.config,axes_cal_board);
            title(axes_cal_board,'Calibration board','FontSize',10);
            drawnow            
            
            debug.plot_res(res_L,colors,alphas,max_res,axes_res_L); 
            title(axes_res_L,'Residuals (left)','FontSize',10); 
            xlabel(axes_res_L,{['mean: [' num2str(mean(res_L{idx_board})) ']'],[' stddev: [' num2str(std(res_L{idx_board})) ']']}, ...
                   'FontSize',8);
            drawnow
            
            debug.plot_res(res_R,colors,alphas,max_res,axes_res_R);
            title(axes_res_R,'Residuals (right)','FontSize',10); 
            xlabel(axes_res_R,{['mean: [' num2str(mean(res_R{idx_board})) ']'],[' stddev: [' num2str(std(res_R{idx_board})) ']']}, ...
                   'FontSize',8);    
            drawnow
            
            debug.plot_cb_img_calib_2D(calib.L, ...
                                       idx_board, ...
                                       axes_board_L);
            title(axes_board_L,'Left board', ...
                  'FontSize',10,'Interpreter','none'); 
            xlabel(axes_board_L,['Path: ' calib.L.extrin(idx_board).cb_img.get_path()], ...
                   'FontSize',8,'Interpreter','none');    
            drawnow
               
            debug.plot_cb_img_calib_2D(calib.R, ...
                                       idx_board, ...
                                       axes_board_R);
            title(axes_board_R,'Right board', ...
                  'FontSize',10,'Interpreter','none'); 
            xlabel(axes_board_R,['Path: ' calib.R.extrin(idx_board).cb_img.get_path()], ...
                   'FontSize',8,'Interpreter','none');    
            drawnow
        catch e      
            if ishandle(f)
                rethrow(e);
            end
        end
    end

    function set_bounds()         
        try      
            % Set name
            set(f,'Name',['Board: ' num2str(idx_board) ' of ' num2str(num_boards) '; mode: ' mode '; (NOTE: press left, right, "w", and "esc" key arrows to toggle)']);

            % Set bounding box
            switch mode
                case 'whole'
                    [l_L, r_L, t_L, b_L] = img_bb(calib.L.extrin(idx_board).cb_img);
                    [l_R, r_R, t_R, b_R] = img_bb(calib.R.extrin(idx_board).cb_img);
                case 'worst'
                    [l_L, r_L, t_L, b_L] = worst_bb(res_L{idx_board},calib.L.extrin(idx_board).board_points_p);
                    [l_R, r_R, t_R, b_R] = worst_bb(res_R{idx_board},calib.R.extrin(idx_board).board_points_p);
            end
            set(axes_board_L,'Xlim',[l_L r_L], ...
                             'Ylim',[t_L b_L]);
            set(axes_board_R,'Xlim',[l_R r_R], ...
                             'Ylim',[t_R b_R]);
        catch e      
            if ishandle(f)
                rethrow(e);
            end
        end
    end
end

function [l, r, t, b] = img_bb(cb_img)
    img_height = cb_img.get_height();
    img_width = cb_img.get_width();
    l = 0.5;
    r = img_width+0.5;
    t = 0.5;
    b = img_height+0.5;
end

function [l, r, t, b] = worst_bb(res,board_points_p)
    zoom_factor = 2;
    [~,max_idx] = max(sum(res.^2,2));
    x_max_res = board_points_p(max_idx,1);
    y_max_res = board_points_p(max_idx,2);
    max_res = max(abs(res(max_idx,:)));
    window = max_res*zoom_factor;
    
    if window < 10
        window = 10;
    end
    
    l = x_max_res - window;
    r = x_max_res + window;
    t = y_max_res - window;
    b = y_max_res + window;
end
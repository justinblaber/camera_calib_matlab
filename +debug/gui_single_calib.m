function gui_single_calib(calib,f)
    % GUI for single calibration
            
    if ~exist('f','var')
        f = figure(); 
    end
    set(f,'Interruptible','off');
    
    % Disable KeyPressFcn until after plotting is complete
    set(f,'KeyPressFcn',@(~,~)drawnow);
                 
    % Initialize parameters
    mode = 'whole';
    idx_board = 1;
    num_boards = length(calib.extrin);                               
    alphas = 0.1*ones(1,num_boards);
    alphas(idx_board) = 1;             
    colors = external.distinguishable_colors(num_boards,{'w','r','k'});
    axes_board = matlab.graphics.axis.Axes.empty();
    res = {};
    for i = 1:num_boards
        board_points_m = alg.p_m(calib.intrin.A, ...
                                 calib.intrin.distortion, ...
                                 calib.extrin(i).rotation, ...
                                 calib.extrin(i).translation, ...
                                 alg.cb_points(calib.config));
        res{i} = board_points_m-calib.extrin(i).board_points_p; %#ok<AGROW>
    end 
    max_res = max(cellfun(@(x)max(abs(x(:))),res));
    
    % Set axes parameters
    padding_height = 0.1;
    padding_width = 0.05;
    extrinsics_height = 0.25;
    extrinsics_width = 0.4;
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

            pos_res = [pos_cal_board(1)+pos_cal_board(3)+padding_width 1-padding_height-res_height 1-(pos_cal_board(1)+pos_cal_board(3))-2*padding_width res_height];
            axes_res = axes('Position',pos_res,'Parent',f);

            pos_board = [pos_res(1) pos_cal_board(2) pos_res(3) pos_res(2)-2*padding_height];
            axes_board = axes('Position',pos_board,'Parent',f);

            % Plot
            debug.plot_single_extrinsic({calib.extrin.rotation}, ...
                                        {calib.extrin.translation}, ...
                                        colors, ...
                                        alphas, ...
                                        calib.config, ...
                                        axes_extrinsics);  
            title(axes_extrinsics,'Extrinsics','FontSize',10); 
            drawnow

            debug.plot_cb_board_info_2D(calib.config,axes_cal_board);
            title(axes_cal_board,'Calibration board','FontSize',10);
            drawnow
            
            debug.plot_res(res,colors,alphas,max_res,axes_res); 
            title(axes_res,'Residuals','FontSize',10); 
            xlabel(axes_res,{['Board mean: [' num2str(mean(res{idx_board})) ']'], ...
                             ['Board stddev: [' num2str(std(res{idx_board})) ']'], ...
                             ['Overall mean: [' num2str(mean(vertcat(res{:}))) ']'], ...
                             ['Overall stddev: [' num2str(std(vertcat(res{:}))) ']']}, ...
                   'FontSize',7);
            drawnow
               
            debug.plot_cb_img_calib_2D(calib, ...
                                       idx_board, ...
                                       axes_board);
            title(axes_board,'Calibration board', ...
                  'FontSize',10,'Interpreter','none'); 
            xlabel(axes_board,{['Path: ' calib.extrin(idx_board).cb_img.get_path()], ...
                               ['Resolution: ' num2str(calib.extrin(idx_board).cb_img.get_width()) ' x ' num2str(calib.extrin(idx_board).cb_img.get_height())]}, ...
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
                    [l, r, t, b] = img_bb(calib.extrin(idx_board).cb_img);
                case 'worst'
                    [l, r, t, b] = worst_bb(res{idx_board},calib.extrin(idx_board).board_points_p);
            end
            set(axes_board,'Xlim',[l r], ...
                           'Ylim',[t b]);
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
function gui_single(cb_imgs,board_points_ps,four_points_ps,A,distortion,rotations,translations,homographies_refine,cal_config,f)
    % GUI for stereo calibration
            
    if ~exist('f','var')
        f = figure(); 
    end
    set(f,'Interruptible','off');
    
    % Disable KeyPressFcn until after plotting is complete
    set(f,'KeyPressFcn',@(~,~)drawnow);
                 
    % Initialize parameters
    idx_board = 1;
    num_boards = length(rotations);                               
    alphas = 0.1*ones(1,num_boards);
    alphas(idx_board) = 1;             
    colors = util.distinguishable_colors(num_boards,{'w','r','k'});
    
    % Set axes parameters
    padding_height = 0.1;
    padding_width = 0.05;
    extrinsics_height = 0.25;
    extrinsics_width = 0.4;
    res_height = extrinsics_height;
    
    % Initialize plot
    plot();
    
    % Set KeyPressFcn callback
    set(f,'KeyPressFcn',@KeyPressFcn);
    
    function KeyPressFcn(~,eventData)   
        try        
            % Disable KeyPressFcn until after this is done
            set(f,'KeyPressFcn',@(~,~)drawnow);
            
            % Set idx_board
            switch eventData.Key
                case 'rightarrow'
                    if idx_board < num_boards
                        idx_board = idx_board+1;
                    else
                        % Set KeyPressFcn callback
                        set(f,'KeyPressFcn',@KeyPressFcn);  
                        return
                    end
                case 'leftarrow'
                    if idx_board > 1
                        idx_board = idx_board-1;
                    else
                        % Set KeyPressFcn callback
                        set(f,'KeyPressFcn',@KeyPressFcn);  
                        return
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
            plot();        

            % Set KeyPressFcn callback
            set(f,'KeyPressFcn',@KeyPressFcn);  
        catch e           
            if ishandle(f)
                rethrow(e);
            end
        end
    end
    
    function plot()  
        try        
            % Clear figure for simplicity
            clf(f);

            % Set name
            set(f,'Name',['Board: ' num2str(idx_board) ' of ' num2str(num_boards) ' (NOTE: press left and right key arrows to toggle)']);

            % Set axes  
            pos_extrinsics = [padding_width 1-padding_height-extrinsics_height extrinsics_width extrinsics_height];
            axes_extrinsics = axes('Position',pos_extrinsics,'Parent',f);

            pos_cal_board = [padding_width padding_height pos_extrinsics(3) pos_extrinsics(2)-2*padding_height];
            axes_cal_board = axes('Position',pos_cal_board,'Parent',f);        

            pos_res = [pos_cal_board(1)+pos_cal_board(3)+padding_width 1-padding_height-res_height 1-(pos_cal_board(1)+pos_cal_board(3))-2*padding_width res_height];
            axes_res = axes('Position',pos_res,'Parent',f);

            pos_board = [pos_res(1) pos_cal_board(2) pos_res(3) pos_res(2)-2*padding_height];
            axes_board = axes('Position',pos_board,'Parent',f);

            % Compute residuals for plots
            res = {};
            for i = 1:num_boards
                board_points_m = alg.p_m(A, ...
                                         distortion, ...
                                         rotations{i}, ...
                                         translations{i}, ...
                                         alg.cb_points(cal_config));
                res{i} = board_points_m-board_points_ps{i};  %#ok<AGROW>
            end 

            % Plot
            debug.plot_single_extrinsic(rotations, ...
                                        translations, ...
                                        colors, ...
                                        alphas, ...
                                        cal_config, ...
                                        axes_extrinsics);  
            title(axes_extrinsics,'Extrinsics','FontSize',10); 

            debug.plot_cb_board_info_2D(cal_config,axes_cal_board);
            title(axes_cal_board,'Calibration board','FontSize',10);

            debug.plot_res(res,colors,alphas,axes_res); 
            title(axes_res,'Residuals','FontSize',10); 
            xlabel(axes_res,{['mean: [' num2str(mean(res{idx_board})) ']'],[' stddev: [' num2str(std(res{idx_board})) ']']}, ...
                   'FontSize',8);
               
            debug.plot_cb_img_info_2D(cb_imgs(idx_board), ...
                                      board_points_ps{idx_board}, ...
                                      four_points_ps{idx_board}, ...
                                      A, ...     
                                      distortion, ...      
                                      rotations{idx_board}, ...
                                      translations{idx_board}, ...                                      
                                      homographies_refine{idx_board}, ...
                                      cal_config, ...
                                      axes_board);
            title(axes_board,'Calibration board', ...
                  'FontSize',10,'Interpreter','none'); 
            xlabel(axes_board,['Path: ' cb_imgs(idx_board).get_path()], ...
                   'FontSize',8,'Interpreter','none'); 
        catch e      
            if ishandle(f)
                rethrow(e);
            end
        end
    end
end
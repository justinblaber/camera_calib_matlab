function gui_stereo_calib(calib,R_s,t_s,f)
    % GUI for stereo calibration
            
    if ~exist('f','var')
        f = figure(); 
    end
    set(f,'Interruptible','off');
    
    % Disable KeyPressFcn until after plotting is complete
    set(f,'KeyPressFcn',@(~,~)drawnow);
                 
    % Initialize parameters
    idx_board = 1;
    num_boards = length(calib.L.extrin);                               
    alphas = 0.1*ones(1,num_boards);
    alphas(idx_board) = 1;             
    colors = external.distinguishable_colors(num_boards,{'w','r','k'});
    
    % Set axes parameters
    padding_height = 0.1;
    padding_width = 0.05;
    extrinsics_height = 0.25;
    extrinsics_width = 0.2;
    res_height = extrinsics_height;
    
    % Initialize plot
    plot_gui();
    
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
            plot_gui();        

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

            % Set name
            set(f,'Name',['Board: ' num2str(idx_board) ' of ' num2str(num_boards) ' (NOTE: press left and right key arrows to toggle)']);

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

            % Compute residuals for plots
            res.L = {};
            res.R = {};
            for i = 1:num_boards
                % Left
                board_points_m_L = alg.p_m(calib.L.intrin.A, ...
                                           calib.L.intrin.distortion, ...
                                           calib.L.extrin(i).rotation, ...
                                           calib.L.extrin(i).translation, ...
                                           alg.cb_points(calib.L.config));
                res.L{i} = board_points_m_L-calib.L.extrin(i).board_points_p;   

                % Right
                board_points_m_R = alg.p_m(calib.R.intrin.A, ...
                                           calib.R.intrin.distortion, ...
                                           R_s*calib.L.extrin(i).rotation, ...
                                           R_s*calib.L.extrin(i).translation+t_s, ...
                                           alg.cb_points(calib.L.config));
                res.R{i} = board_points_m_R-calib.R.extrin(i).board_points_p;  
            end 

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

            debug.plot_cb_board_info_2D(calib.L.config,axes_cal_board);
            title(axes_cal_board,'Calibration board','FontSize',10);

            debug.plot_res(res.L,colors,alphas,axes_res_L); 
            title(axes_res_L,'Residuals (left)','FontSize',10); 
            xlabel(axes_res_L,{['mean: [' num2str(mean(res.L{idx_board})) ']'],[' stddev: [' num2str(std(res.L{idx_board})) ']']}, ...
                   'FontSize',8);

            debug.plot_res(res.R,colors,alphas,axes_res_R);
            title(axes_res_R,'Residuals (right)','FontSize',10); 
            xlabel(axes_res_R,{['mean: [' num2str(mean(res.R{idx_board})) ']'],[' stddev: [' num2str(std(res.R{idx_board})) ']']}, ...
                   'FontSize',8);    

            debug.plot_cb_img_calib_2D(calib.L, ...
                                       idx_board, ...
                                       axes_board_L);
            title(axes_board_L,'Left board', ...
                  'FontSize',10,'Interpreter','none'); 
            xlabel(axes_board_L,['Path: ' calib.L.extrin(idx_board).cb_img.get_path()], ...
                   'FontSize',8,'Interpreter','none');    

            debug.plot_cb_img_calib_2D(calib.R, ...
                                       idx_board, ...
                                       axes_board_R);
            title(axes_board_R,'Right board', ...
                  'FontSize',10,'Interpreter','none'); 
            xlabel(axes_board_R,['Path: ' calib.R.extrin(idx_board).cb_img.get_path()], ...
                   'FontSize',8,'Interpreter','none');    
        catch e      
            if ishandle(f)
                rethrow(e);
            end
        end
    end
end

function gui_four_points_detect(four_points_ps,four_points_debugs,cb_imgs,f)
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
    
    % Set axes parameters
    padding_height = 0.1;
    padding_width = 0.05;
    patches_width = 0.3;
    
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
            set(f,'Name',['Board: ' num2str(idx_board) ' of ' num2str(num_boards) '; mode: ' mode '; (NOTE: press left, right, "1", "2", "3", "4", "w", and "esc" key arrows to toggle)']);

            % Set axes  
            pos_cal_board = [padding_width padding_height 1-patches_width-3*padding_width 1-2*padding_height];
            axes_cal_board = axes('Position',pos_cal_board,'Parent',f);  

            axes_patches = matlab.graphics.axis.Axes.empty();
            single_patch_height = (1-5*padding_height)/4;
            single_patch_width = (patches_width-padding_width)/2;
            for i = 1:4
                for j = 1:2
                    pos_patch = [pos_cal_board(1)+pos_cal_board(3)+padding_width+(j-1)*(single_patch_width+padding_width) ...
                                 padding_height+(4-i)*(single_patch_height+padding_height)  ...
                                 single_patch_width ...
                                 single_patch_height];
                    axes_patches(i,j) = axes('Position',pos_patch,'Parent',f);
                end
            end

            % Plot
            imshow(cb_imgs(idx_board).get_gs(),[],'Parent',axes_cal_board)
            title(axes_cal_board,'Blobs, ellipses, and four points','FontSize',10); 
            xlabel(axes_cal_board,['Path: ' cb_imgs(idx_board).get_path()], ...
                   'FontSize',8,'Interpreter','none'); 
            axes(axes_cal_board);
            hold(axes_cal_board,'on');
            for i = 1:length(four_points_debugs(idx_board).blobs)
                external.ellipse(four_points_debugs(idx_board).blobs(i).r, ...
                                 four_points_debugs(idx_board).blobs(i).r, ...
                                 0, ...
                                 four_points_debugs(idx_board).blobs(i).x, ...
                                 four_points_debugs(idx_board).blobs(i).y, ...
                                 'r');  
            end
            for i = 1:length(four_points_debugs(idx_board).ellipses)
                external.ellipse(four_points_debugs(idx_board).ellipses(i).r1, ...
                                 four_points_debugs(idx_board).ellipses(i).r2, ...
                                 four_points_debugs(idx_board).ellipses(i).rot, ...
                                 four_points_debugs(idx_board).ellipses(i).x, ...
                                 four_points_debugs(idx_board).ellipses(i).y, ...
                                 'g');  
            end
            plot(four_points_ps{idx_board}(:,1),four_points_ps{idx_board}(:,2),'-mo','MarkerSize',8, ...
                 'parent',axes_cal_board);
            hold(axes_cal_board,'off');

            for i = 1:4
                imshow(four_points_debugs(idx_board).patch_matches(i).patch,[],'Parent',axes_patches(i,1));
                title(axes_patches(i,1),['Patch ' num2str(i) ' sampled'],'FontSize',7);
                imshow(four_points_debugs(idx_board).patch_matches(i).template,[],'Parent',axes_patches(i,2));
                title(axes_patches(i,2),['Patch ' num2str(i) ' template'],'FontSize',7);
            end
            
            scale_factor = 10;
            switch mode
                case 'whole'
                    l = 0.5;
                    r = cb_imgs(1).get_width()+0.5;
                    t = 0.5;
                    b = cb_imgs(1).get_height()+0.5;
                case '1' 
                    ellipse = four_points_debugs(idx_board).patch_matches(1).ellipse;
                    width_ellipse = sqrt(ellipse.r1^2*cos(ellipse.rot)^2 + ellipse.r2^2*sin(ellipse.rot)^2);
                    height_ellipse = sqrt(ellipse.r1^2*sin(ellipse.rot)^2 + ellipse.r2^2*cos(ellipse.rot)^2); 
                    l = ellipse.x-scale_factor*width_ellipse;
                    r = ellipse.x+scale_factor*width_ellipse;
                    t = ellipse.y-scale_factor*height_ellipse;
                    b = ellipse.y+scale_factor*height_ellipse;
                case '2'
                    ellipse = four_points_debugs(idx_board).patch_matches(2).ellipse;
                    width_ellipse = sqrt(ellipse.r1^2*cos(ellipse.rot)^2 + ellipse.r2^2*sin(ellipse.rot)^2);
                    height_ellipse = sqrt(ellipse.r1^2*sin(ellipse.rot)^2 + ellipse.r2^2*cos(ellipse.rot)^2); 
                    l = ellipse.x-scale_factor*width_ellipse;
                    r = ellipse.x+scale_factor*width_ellipse;
                    t = ellipse.y-scale_factor*height_ellipse;
                    b = ellipse.y+scale_factor*height_ellipse;
                case '3'
                    ellipse = four_points_debugs(idx_board).patch_matches(3).ellipse;
                    width_ellipse = sqrt(ellipse.r1^2*cos(ellipse.rot)^2 + ellipse.r2^2*sin(ellipse.rot)^2);
                    height_ellipse = sqrt(ellipse.r1^2*sin(ellipse.rot)^2 + ellipse.r2^2*cos(ellipse.rot)^2); 
                    l = ellipse.x-scale_factor*width_ellipse;
                    r = ellipse.x+scale_factor*width_ellipse;
                    t = ellipse.y-scale_factor*height_ellipse;
                    b = ellipse.y+scale_factor*height_ellipse;
                case '4'
                    ellipse = four_points_debugs(idx_board).patch_matches(4).ellipse;
                    width_ellipse = sqrt(ellipse.r1^2*cos(ellipse.rot)^2 + ellipse.r2^2*sin(ellipse.rot)^2);
                    height_ellipse = sqrt(ellipse.r1^2*sin(ellipse.rot)^2 + ellipse.r2^2*cos(ellipse.rot)^2); 
                    l = ellipse.x-scale_factor*width_ellipse;
                    r = ellipse.x+scale_factor*width_ellipse;
                    t = ellipse.y-scale_factor*height_ellipse;
                    b = ellipse.y+scale_factor*height_ellipse;
                case 'worst'
                    % Get the worst patch
                    [~,idx] = min([four_points_debugs(idx_board).patch_matches.cc_val]);
                    ellipse = four_points_debugs(idx_board).patch_matches(idx).ellipse;
                    width_ellipse = sqrt(ellipse.r1^2*cos(ellipse.rot)^2 + ellipse.r2^2*sin(ellipse.rot)^2);
                    height_ellipse = sqrt(ellipse.r1^2*sin(ellipse.rot)^2 + ellipse.r2^2*cos(ellipse.rot)^2); 
                    l = ellipse.x-scale_factor*width_ellipse;
                    r = ellipse.x+scale_factor*width_ellipse;
                    t = ellipse.y-scale_factor*height_ellipse;
                    b = ellipse.y+scale_factor*height_ellipse;
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
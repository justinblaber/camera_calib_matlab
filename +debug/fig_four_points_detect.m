function fig_four_points_detect(four_points_ps,debug,cb_img,f)
    % figure for debugging four point detection
            
    if ~exist('f','var')
        f = figure(); 
    end
    clf(f);
                  
    % Set axes parameters
    padding_height = 0.1;
    padding_width = 0.05;
    patches_width = 0.3;
    
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
    imshow(cb_img.get_gs(),[],'Parent',axes_cal_board)
    title(axes_cal_board,'Blobs, ellipses, and four points','FontSize',10); 
    xlabel(axes_cal_board,['Path: ' cb_img.get_path()], ...
           'FontSize',8,'Interpreter','none'); 
    axes(axes_cal_board);
    hold(axes_cal_board,'on');
    for i = 1:length(debug.blobs)
        util.ellipse(debug.blobs(i).r,debug.blobs(i).r,0,debug.blobs(i).x,debug.blobs(i).y,'r');  
        util.ellipse(debug.ellipses(i).r1,debug.ellipses(i).r2,debug.ellipses(i).rot, ...
                     debug.ellipses(i).x,debug.ellipses(i).y,'g');  
    end
    plot(four_points_ps(:,1),four_points_ps(:,2),'-mo','MarkerSize',8, ...
         'parent',axes_cal_board);
    hold(axes_cal_board,'off');

    for i = 1:4
        for j = 1:2
            imshow(debug.patch_matches{i,j},[],'Parent',axes_patches(i,j));
            if j == 1
                title(axes_patches(i,j),['Patch ' num2str(i) ' sampled'],'FontSize',7);
            else
                title(axes_patches(i,j),['Patch ' num2str(i) ' template'],'FontSize',7);
            end
        end
    end
end
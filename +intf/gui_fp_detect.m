function gui_fp_detect(p_fpss, img_cbs, debug_fp, calib_config, f)
    % GUI for debugging four point detection

    if ~exist('f', 'var')
        f = figure();
    end
    set(f, 'Interruptible', 'off');

    % Disable KeyPressFcn until after plotting is complete
    set(f, 'KeyPressFcn', @(~, ~)drawnow);

    % Get number of cameras and boards
    num_cams = size(p_fpss, 2);
    num_boards = size(p_fpss, 1);

    % Initialize parameters
    mode = 'whole';
    idx_board = 1;
    axes_cbs = matlab.graphics.axis.Axes.empty();

    % Get scale factor
    if isnan(calib_config.fp_detect_array_min_size)
        sf = 1;
    else
        sf = calib_config.fp_detect_array_min_size/min(img_cbs(1).get_size());
    end

    % Set axes parameters
    padding_height = 0.075;
    padding_width = 0.025;
    height_cb = 0.65;

    % Initialize plot
    plot_gui();

    % Set bounds
    set_bounds();

    % Set KeyPressFcn callback
    set(f, 'KeyPressFcn', @KeyPressFcn);

    function KeyPressFcn(~, eventData)
        try
            % Disable KeyPressFcn until after this is done
            set(f, 'KeyPressFcn', @(~, ~)drawnow);

            % Set idx_board
            replot = false;
            switch eventData.Key
                case 'rightarrow'
                    if idx_board < num_boards
                        idx_board = idx_board+1;
                        replot = true;
                    else
                        % Set KeyPressFcn callback
                        set(f, 'KeyPressFcn', @KeyPressFcn);
                        return
                    end
                case 'leftarrow'
                    if idx_board > 1
                        idx_board = idx_board-1;
                        replot = true;
                    else
                        % Set KeyPressFcn callback
                        set(f, 'KeyPressFcn', @KeyPressFcn);
                        return
                    end
                case 'escape'
                    mode = 'whole';
                case '1'
                    if strcmp(mode, '1')
                        mode = 'whole';
                    else
                        mode = '1';
                    end
                case '2'
                    if strcmp(mode, '2')
                        mode = 'whole';
                    else
                        mode = '2';
                    end
                case '3'
                    if strcmp(mode, '3')
                        mode = 'whole';
                    else
                        mode = '3';
                    end
                case '4'
                    if strcmp(mode, '4')
                        mode = 'whole';
                    else
                        mode = '4';
                    end
                case 'w'
                    if strcmp(mode, 'worst')
                        mode = 'whole';
                    else
                        mode = 'worst';
                    end
                otherwise
                    % Set KeyPressFcn callback
                    set(f, 'KeyPressFcn', @KeyPressFcn);
                    return
            end

            % Replot
            if replot
                plot_gui();
            end

            % Set bounds
            set_bounds();

            % Set KeyPressFcn callback
            set(f, 'KeyPressFcn', @KeyPressFcn);
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
            height_patch = 1-height_cb-3*padding_height;
            width_patch = (1-(4*num_cams+1)*padding_width)/(4*num_cams);
            width_cb = (1-(num_cams+1)*padding_width)/num_cams;

            axes_patches = matlab.graphics.axis.Axes.empty();
            for i = 1:num_cams
                pos_cb = [i*padding_width+(i-1)*width_cb 2*padding_height+height_patch width_cb height_cb];
                axes_cbs(i) = axes('Position', pos_cb, 'Parent', f);

                for j = 1:4
                    pos_patch = [pos_cb(1)+(j-1)*(width_patch+padding_width) ...
                                 padding_height  ...
                                 width_patch ...
                                 height_patch];
                    axes_patches(j, i) = axes('Position', pos_patch, 'Parent', f);
                end
            end

            % Plot Calibration board -------------------------------------%

            for i = 1:num_cams
                % Resize array
                array = img_cbs(idx_board, i).get_array_gs();
                array = imresize(array, sf);

                % Rescale four points
                p_fps = alg.p2imresize(p_fpss{idx_board, i}, sf);

                % Plot four points
                debug.plot_fp(array, p_fps, axes_cbs(i));

                if any(strcmp(calib_config.fp_detector, {'LoG', 'thresh'}))
                    % Plot blobs and ellipses
                    axes(axes_cbs(i)); %#ok<LAXES>
                    hold(axes_cbs(i), 'on');
                    for j = 1:size(debug_fp{idx_board, i}.blobs, 1)
                        external.ellipse(debug_fp{idx_board, i}.blobs(j, 3), ...
                                         debug_fp{idx_board, i}.blobs(j, 4), ...
                                         debug_fp{idx_board, i}.blobs(j, 5), ...
                                         debug_fp{idx_board, i}.blobs(j, 1), ...
                                         debug_fp{idx_board, i}.blobs(j, 2), ...
                                         'r');
                    end
                    for j = 1:size(debug_fp{idx_board, i}.ellipses, 1)
                        external.ellipse(debug_fp{idx_board, i}.ellipses(j, 3), ...
                                         debug_fp{idx_board, i}.ellipses(j, 4), ...
                                         debug_fp{idx_board, i}.ellipses(j, 5), ...
                                         debug_fp{idx_board, i}.ellipses(j, 1), ...
                                         debug_fp{idx_board, i}.ellipses(j, 2), ...
                                         'g');
                    end
                    % Remove hold
                    hold(axes_cbs(i), 'off');
                end

                title(axes_cbs(i), 'Four point debugging info', 'FontSize', 10);
                xlabel(axes_cbs(i), ...
                       ['Name: ' img_cbs(idx_board, i).get_name()], ...
                       'FontSize', 8, 'Interpreter', 'none');
            end

            % Plot four point patches ------------------------------------%

            for i = 1:num_cams
                for j = 1:4
                    if any(strcmp(calib_config.fp_detector, {'LoG', 'thresh'}))
                        debug.plot_patch_match(debug_fp{idx_board, i}.patch_matches(j).patch, ...
                                               debug_fp{idx_board, i}.patch_matches(j).template, ...
                                               axes_patches(j, i));
                        title(axes_patches(j, i), [num2str(j) ' (CC val: ' num2str(debug_fp{idx_board, i}.patch_matches(j).val_cc) ')'], 'FontSize', 7);
                    end
                end
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
            set(f, 'Name', ['Board: ' num2str(idx_board) ' of ' num2str(num_boards) '; mode: ' mode '; (NOTE: press left, right, "1", "2", "3", "4", "w", and "esc" key arrows to toggle)']);

            if any(strcmp(calib_config.fp_detector, {'LoG', 'thresh'}))
                % Set bounding box
                for i = 1:num_cams
                    switch mode
                        case 'whole'
                            bb = bb_img(img_cbs(idx_board, i), sf);
                        case '1'
                            bb = bb_ellipse(debug_fp{idx_board, i}.patch_matches(1).ellipse);
                        case '2'
                            bb = bb_ellipse(debug_fp{idx_board, i}.patch_matches(2).ellipse);
                        case '3'
                            bb = bb_ellipse(debug_fp{idx_board, i}.patch_matches(3).ellipse);
                        case '4'
                            bb = bb_ellipse(debug_fp{idx_board, i}.patch_matches(4).ellipse);
                        case 'worst'
                            % Get the worst patch
                            [~, idx] = min([debug_fp{idx_board, i}.patch_matches.val_cc]);
                            bb = bb_ellipse(debug_fp{idx_board, i}.patch_matches(idx).ellipse);
                    end

                    set(axes_cbs(i), 'Xlim', bb(:, 1), 'Ylim', bb(:, 2));
                end
            end
        catch e
            if ishandle(f)
                rethrow(e);
            end
        end
    end
end

function bb = bb_img(img_cb, sf)
    % Get size of image
    size_img = img_cb.get_size();

    % Set bounding box
    bb = [0.5 0.5; ...
          size_img(2)*sf+0.5 size_img(1)*sf+0.5];
end

function bb = bb_ellipse(e)
    % Apply scale factor to ellipse
    sf = 10;
    e(3:4) = sf*e(3:4);

    % Get bounding box
    bb = alg.bb_ellipse(e);

    % Get height and width
    h = bb(2, 2) - bb(1, 2);
    w = bb(2, 1) - bb(1, 1);

    % Set bounding box to max size to make it square
    max_size = max(h, w);
    bb = [e(1)-max_size/2 e(2)-max_size/2;
          e(1)+max_size/2 e(2)+max_size/2];
end

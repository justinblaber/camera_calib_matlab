function gui_single_fp_detect_LoG(p_fpss, debug_single_fp_detect_LoG, img_cbs, calib_config, f)
    % GUI for debugging four point LoG detection

    if ~exist('f', 'var')
        f = figure();
    end
    set(f, 'Interruptible', 'off');

    % Disable KeyPressFcn until after plotting is complete
    set(f, 'KeyPressFcn', @(~, ~)drawnow);

    % Initialize parameters
    mode = 'whole';
    idx_board = 1;
    num_boards = numel(img_cbs);
    axes_cb = matlab.graphics.axis.Axes.empty();

    % Set axes parameters
    padding_height = 0.075;
    padding_width = 0.025;
    width_cb = 0.7;

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
            height_patch = (1-5*padding_height)/4;
            width_patch = 1-width_cb-3*padding_width;

            pos_cb = [1-width_cb-padding_width padding_height width_cb 1-2*padding_height];
            axes_cb = axes('Position', pos_cb, 'Parent', f);

            axes_patches = matlab.graphics.axis.Axes.empty();
            for i = 1:4
                pos_patch = [padding_width ...
                             padding_height+(4-i)*(height_patch+padding_height)  ...
                             width_patch ...
                             height_patch];
                axes_patches(i) = axes('Position', pos_patch, 'Parent', f);
            end

            % Plot debugging info
            debug.plot_debug_single_fp_detect_LoG(p_fpss{idx_board}, ...
                                                  debug_single_fp_detect_LoG(idx_board), ...
                                                  img_cbs(idx_board), ...
                                                  calib_config, ...
                                                  axes_cb);
            title(axes_cb, 'Blobs, ellipses, and four points', 'FontSize', 10);
            xlabel(axes_cb, ['Path: ' img_cbs(idx_board).get_path()], ...
                   'FontSize', 8, 'Interpreter', 'none');

            % Plot patches
            for i = 1:4
                debug.plot_patch_match(debug_single_fp_detect_LoG(idx_board).patch_matches(i), ...
                                       i, ...
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
            set(f, 'Name', ['Board: ' num2str(idx_board) ' of ' num2str(num_boards) '; mode: ' mode '; (NOTE: press left, right, "1", "2", "3", "4", "w", and "esc" key arrows to toggle)']);

            % Set bounding box
            switch mode
                case 'whole'
                    bb = bb_img(img_cbs(idx_board), calib_config);
                case '1'
                    bb = bb_ellipse(debug_single_fp_detect_LoG(idx_board).patch_matches(1).ellipse);
                case '2'
                    bb = bb_ellipse(debug_single_fp_detect_LoG(idx_board).patch_matches(2).ellipse);
                case '3'
                    bb = bb_ellipse(debug_single_fp_detect_LoG(idx_board).patch_matches(3).ellipse);
                case '4'
                    bb = bb_ellipse(debug_single_fp_detect_LoG(idx_board).patch_matches(4).ellipse);
                case 'worst'
                    % Get the worst patch
                    [~, idx] = min([debug_single_fp_detect_LoG(idx_board).patch_matches.val_cc]);
                    bb = bb_ellipse(debug_single_fp_detect_LoG(idx_board).patch_matches(idx).ellipse);
            end

            set(axes_cb, 'Xlim', bb(:, 1), 'Ylim', bb(:, 2));
        catch e
            if ishandle(f)
                rethrow(e);
            end
        end
    end
end

function bb = bb_img(img_cb, calib_config)
    % Get size of image
    height_img = img_cb.get_height();
    width_img = img_cb.get_width();

    % Get scale factor
    if isnan(calib_config.fp_detect_array_min_size)
        sf = 1;
    else
        sf = calib_config.fp_detect_array_min_size/min([height_img width_img]);
    end

    % Set bounding box
    bb = [0.5 0.5; ...
          width_img*sf+0.5 height_img*sf+0.5];
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

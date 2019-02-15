function gui_stereo_calib(calib, f)
    % GUI for stereo calibration

    if ~exist('f', 'var')
        f = figure();
    end
    set(f, 'Interruptible', 'off');

    % Disable KeyPressFcn until after plotting is complete
    set(f, 'KeyPressFcn', @(~, ~)drawnow);

    % Initialize parameters
    mode = 'whole';
    idx_board = 1;
    num_boards = numel(calib.L.extrin);
    alphas = 0.1*ones(1, num_boards);
    alphas(idx_board) = 1;
    colors = external.distinguishable_colors(num_boards, {'w', 'r', 'k'});
    axes_calib_cb_img_L = matlab.graphics.axis.Axes.empty();
    axes_calib_cb_img_R = matlab.graphics.axis.Axes.empty();

    % Get residuals
    res_L = {};
    res_R = {};
    for i = 1:num_boards
        % Left
        res_L{i} = calib.L.extrin(i).p_cb_p_d_ms - calib.L.extrin(i).p_cb_p_ds; %#ok<AGROW>
        res_L{i} = res_L{i}(calib.L.extrin(i).idx_valid, :); %#ok<AGROW>

        % Right
        res_R{i} = calib.R.extrin(i).p_cb_p_d_ms - calib.R.extrin(i).p_cb_p_ds; %#ok<AGROW>
        res_R{i} = res_R{i}(calib.R.extrin(i).idx_valid, :); %#ok<AGROW>
    end
    max_res = max(max(vertcat(res_L{:}, res_R{:})));

    % Set axes parameters
    padding_height = 0.1;
    padding_width = 0.05;
    height_extrinsics = 0.25;
    width_extrinsics = 0.2;
    height_res = height_extrinsics;

    % Initialize plot
    plot_gui();

    % Set bounds
    set_bounds()

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

            % Set alphas
            alphas = 0.1*ones(1, num_boards);
            alphas(idx_board) = 1;

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
            pos_extrinsics = [padding_width 1-padding_height-height_extrinsics width_extrinsics height_extrinsics];
            axes_extrinsics = axes('Position', pos_extrinsics, 'Parent', f);

            pos_cb_class = [padding_width padding_height pos_extrinsics(3) pos_extrinsics(2)-2*padding_height];
            axes_cb_class = axes('Position', pos_cb_class, 'Parent', f);

            pos_res_L = [pos_cb_class(1)+pos_cb_class(3)+padding_width 1-padding_height-height_res (1-(pos_cb_class(1)+pos_cb_class(3))-3*padding_width)/2 height_res];
            axes_res_L = axes('Position', pos_res_L, 'Parent', f);

            pos_res_R = [pos_res_L(1)+pos_res_L(3)+padding_width pos_res_L(2) pos_res_L(3) pos_res_L(4)];
            axes_res_R = axes('Position', pos_res_R, 'Parent', f);

            pos_calib_cb_img_L = [pos_res_L(1) pos_cb_class(2) pos_res_L(3) pos_res_L(2)-2*padding_height];
            axes_calib_cb_img_L = axes('Position', pos_calib_cb_img_L, 'Parent', f);

            pos_calib_cb_img_R = [pos_res_R(1) pos_cb_class(2) pos_res_R(3) pos_res_R(2)-2*padding_height];
            axes_calib_cb_img_R = axes('Position', pos_calib_cb_img_R, 'Parent', f);

            % Plot extrinsics --------------------------------------------%

            Rs.L = {calib.L.extrin.R};
            Rs.R = {calib.R.extrin.R};
            ts.L = {calib.L.extrin.t};
            ts.R = {calib.R.extrin.t};
            debug.plot_stereo_extrinsics(Rs, ...
                                         ts, ...
                                         calib.R_s, ...
                                         calib.t_s, ...
                                         colors, ...
                                         alphas, ...
                                         calib.config, ...
                                         axes_extrinsics);
            title(axes_extrinsics, 'Extrinsics', 'FontSize', 10);
            drawnow

            % Plot residuals ---------------------------------------------%

            debug.plot_res(res_L, colors, alphas, max_res, axes_res_L);
            title(axes_res_L, 'Residuals (left)', 'FontSize', 10);
            xlabel(axes_res_L, {['Board mean: [' num2str(mean(res_L{idx_board})) ']'], ...
                                ['Board stddev: [' num2str(std(res_L{idx_board})) ']'], ...
                                ['Overall mean: [' num2str(mean(vertcat(res_L{:}))) ']'], ...
                                ['Overall stddev: [' num2str(std(vertcat(res_L{:}))) ']']}, ...
                   'FontSize', 7);
            drawnow

            debug.plot_res(res_R, colors, alphas, max_res, axes_res_R);
            title(axes_res_R, 'Residuals (right)', 'FontSize', 10);
            xlabel(axes_res_R, {['Board mean: [' num2str(mean(res_R{idx_board})) ']'], ...
                                ['Board stddev: [' num2str(std(res_R{idx_board})) ']'], ...
                                ['Overall mean: [' num2str(mean(vertcat(res_R{:}))) ']'], ...
                                ['Overall stddev: [' num2str(std(vertcat(res_R{:}))) ']']}, ...
                   'FontSize', 7);
            drawnow

            % Plot calibration board class -------------------------------%

            debug.plot_cb_class(calib.config.cb_class, axes_cb_class);
            title(axes_cb_class, 'Calibration board', 'FontSize', 10);
            drawnow

            % Plot calibrated board image --------------------------------%

            debug.plot_calib_cb_img(calib.L.extrin(idx_board), ...
                                    calib.L.intrin, ...
                                    axes_calib_cb_img_L);
            title(axes_calib_cb_img_L, 'Left board', ...
                  'FontSize', 10, 'Interpreter', 'none');
            xlabel(axes_calib_cb_img_L, {['Path: ' calib.L.extrin(idx_board).img_cb.get_path()], ...
                                         ['Resolution: ' num2str(calib.L.extrin(idx_board).img_cb.get_width()) ' x ' num2str(calib.L.extrin(idx_board).img_cb.get_height())]}, ...
                   'FontSize', 8, 'Interpreter', 'none');
            drawnow

            debug.plot_calib_cb_img(calib.R.extrin(idx_board), ...
                                    calib.R.intrin, ...
                                    axes_calib_cb_img_R);
            title(axes_calib_cb_img_R, 'Right board', ...
                  'FontSize', 10, 'Interpreter', 'none');
            xlabel(axes_calib_cb_img_R, {['Path: ' calib.R.extrin(idx_board).img_cb.get_path()], ...
                                         ['Resolution: ' num2str(calib.R.extrin(idx_board).img_cb.get_width()) ' x ' num2str(calib.R.extrin(idx_board).img_cb.get_height())]}, ...
                   'FontSize', 8, 'Interpreter', 'none');
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
            set(f, 'Name', ['Board: ' num2str(idx_board) ' of ' num2str(num_boards) '; mode: ' mode '; (NOTE: press left, right, "w", and "esc" key arrows to toggle)']);

            % Set bounding box
            switch mode
                case 'whole'
                    bb_L = bb_img(calib.L.extrin(idx_board).img_cb);
                    bb_R = bb_img(calib.R.extrin(idx_board).img_cb);
                case 'worst'
                    bb_L = bb_worst(res_L{idx_board}, calib.L.extrin(idx_board).p_cb_p_ds(calib.L.extrin(idx_board).idx_valid, :));
                    bb_R = bb_worst(res_R{idx_board}, calib.R.extrin(idx_board).p_cb_p_ds(calib.R.extrin(idx_board).idx_valid, :));
            end

            set(axes_calib_cb_img_L, 'Xlim', bb_L(:, 1), 'Ylim', bb_L(:, 2));
            set(axes_calib_cb_img_R, 'Xlim', bb_R(:, 1), 'Ylim', bb_R(:, 2));
        catch e
            if ishandle(f)
                rethrow(e);
            end
        end
    end
end

function bb = bb_img(img_cb)
    % Get size of image
    height_img = img_cb.get_height();
    width_img = img_cb.get_width();

    % Set bounding box
    bb = [0.5 0.5; ...
          width_img+0.5 height_img+0.5];
end

function bb = bb_worst(res, p_cb_p_ds)
    % Find location of max residual
    [~, idx_max] = max(sum(res.^2, 2));
    x_max_res = p_cb_p_ds(idx_max, 1);
    y_max_res = p_cb_p_ds(idx_max, 2);
    max_res = max(abs(res(idx_max, :)));

    % Get half window
    sf = 2;
    hw = max_res*sf;
    if hw < 10
        hw = 10;
    end

    % Set bounding box centered around max residual
    bb = [x_max_res-hw y_max_res-hw; ...
          x_max_res+hw y_max_res+hw];
end

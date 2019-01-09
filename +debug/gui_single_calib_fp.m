function gui_single_calib_fp(calib, f)
    % GUI for single calibration

    if ~exist('f', 'var')
        f = figure();
    end
    set(f, 'Interruptible', 'off');

    % Disable KeyPressFcn until after plotting is complete
    set(f, 'KeyPressFcn', @(~, ~)drawnow);

    % Initialize parameters
    mode = 'whole';
    idx_board = 1;
    num_boards = numel(calib.extrin);
    alphas = 0.1*ones(1, num_boards);
    alphas(idx_board) = 1;
    colors = external.distinguishable_colors(num_boards, {'w', 'r', 'k'});
    axes_cb_img = matlab.graphics.axis.Axes.empty();

    % Get residuals
    res = {};
    for i = 1:num_boards
        res{i} = calib.extrin(i).p_cb_p_d_ms - calib.extrin(i).p_cb_p_ds; %#ok<AGROW>
        res{i} = res{i}(calib.extrin(i).idx_valid, :); %#ok<AGROW>
    end
    max_res = max(max(vertcat(res{:})));

    % Set axes parameters
    padding_height = 0.1;
    padding_width = 0.05;
    height_extrinsics = 0.25;
    width_extrinsics = 0.4;
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

            pos_cb_info = [padding_width padding_height pos_extrinsics(3) pos_extrinsics(2)-2*padding_height];
            axes_cb_info = axes('Position', pos_cb_info, 'Parent', f);

            pos_res = [pos_cb_info(1)+pos_cb_info(3)+padding_width 1-padding_height-height_res 1-(pos_cb_info(1)+pos_cb_info(3))-2*padding_width height_res];
            axes_res = axes('Position', pos_res, 'Parent', f);

            pos_cb_img = [pos_res(1) pos_cb_info(2) pos_res(3) pos_res(2)-2*padding_height];
            axes_cb_img = axes('Position', pos_cb_img, 'Parent', f);

            % Plot extrinsics --------------------------------------------%

            Rs = {calib.extrin.R};
            ts = {calib.extrin.t};
            debug.plot_single_extrinsics(Rs, ...
                                         ts, ...
                                         colors, ...
                                         alphas, ...
                                         calib.config, ...
                                         axes_extrinsics);
            title(axes_extrinsics, 'Extrinsics', 'FontSize', 10);
            drawnow

            % Plot calibration board info --------------------------------%

            debug.plot_cb_info_fp(calib.config, axes_cb_info);
            title(axes_cb_info, 'Calibration board', 'FontSize', 10);
            drawnow

            % Plot residuals ---------------------------------------------%

            debug.plot_res(res, colors, alphas, max_res, axes_res);
            title(axes_res, 'Residuals', 'FontSize', 10);
            xlabel(axes_res, {['Board mean: [' num2str(mean(res{idx_board})) ']'], ...
                              ['Board stddev: [' num2str(std(res{idx_board})) ']'], ...
                              ['Overall mean: [' num2str(mean(vertcat(res{:}))) ']'], ...
                              ['Overall stddev: [' num2str(std(vertcat(res{:}))) ']']}, ...
                   'FontSize', 7);
            drawnow

            % Plot calibration board image -------------------------------%

            debug.plot_cb_img_fp(calib.extrin(idx_board).img_cb.get_array_gs(), ...
                                 calib.intrin.A, ...
                                 calib.extrin(idx_board).p_fp_p_ds, ...
                                 calib.extrin(idx_board).p_cb_p_ds(calib.extrin(idx_board).idx_valid, :), ...
                                 calib.extrin(idx_board).p_cb_p_d_ms(calib.extrin(idx_board).idx_valid, :), ...
                                 axes_cb_img);
            title(axes_cb_img, 'Calibration board', ...
                  'FontSize', 10, 'Interpreter', 'none');
            xlabel(axes_cb_img, {['Path: ' calib.extrin(idx_board).img_cb.get_path()], ...
                                 ['Resolution: ' num2str(calib.extrin(idx_board).img_cb.get_width()) ' x ' num2str(calib.extrin(idx_board).img_cb.get_height())]}, ...
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
                    bb = bb_img(calib.extrin(idx_board).img_cb);
                case 'worst'
                    bb = bb_worst(res{idx_board}, calib.extrin(idx_board).p_cb_p_ds(calib.extrin(idx_board).idx_valid, :));
            end

            set(axes_cb_img, 'Xlim', bb(:, 1), 'Ylim', bb(:, 2));
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

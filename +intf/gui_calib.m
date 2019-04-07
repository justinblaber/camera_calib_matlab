function gui_calib(calib, f)
    % Calibration gui

    if ~exist('f', 'var')
        f = figure();
    end
    set(f, 'Interruptible', 'off');

    % Disable KeyPressFcn until after plotting is complete
    set(f, 'KeyPressFcn', @(~, ~)drawnow);

    % Get number of cameras and boards
    num_cams = numel(calib.cam);
    num_boards = numel(calib.cam(1).extrin);

    % Initialize parameters
    mode = 'whole';
    idx_board = 1;
    alphas = 0.1*ones(1, num_boards);
    alphas(idx_board) = 1;
    colors = external.distinguishable_colors(num_boards, {'w', 'r', 'k'});
    axes_calib_cb_imgs = matlab.graphics.axis.Axes.empty();

    % Get residuals
    ress = cell(num_boards, num_cams);
    for i = 1:num_cams %#ok<FXUP>
        for j = 1:num_boards
            ress{j, i} = calib.cam(i).extrin(j).p_cb_p_d_ms - calib.cam(i).extrin(j).p_cb_p_ds;
            ress{j, i} = ress{j, i}(calib.cam(i).extrin(j).idx_valid, :);
        end
    end
    max_res = max(max(vertcat(ress{:})));

    % Set axes parameters
    padding_height = 0.1;
    padding_width = 0.05;
    height_extrinsics = 0.25;
    width_extrinsics = 0.25;
    height_res = height_extrinsics;
    width_res = (1-width_extrinsics-(num_cams+2)*padding_width)/num_cams;

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

            pos_cb_geom = [padding_width padding_height pos_extrinsics(3) 1-height_extrinsics-3*padding_height];
            axes_cb_geom = axes('Position', pos_cb_geom, 'Parent', f);

            axes_ress = matlab.graphics.axis.Axes.empty();
            for i = 1:num_cams %#ok<FXUP>
                pos_res = [pos_cb_geom(1)+pos_cb_geom(3)+i*padding_width+(i-1)*width_res pos_extrinsics(2) width_res height_res];
                axes_ress(i) = axes('Position', pos_res, 'Parent', f);

                pos_calib_cb_img = [pos_res(1) pos_cb_geom(2) pos_res(3) pos_cb_geom(4)];
                axes_calib_cb_imgs(i) = axes('Position', pos_calib_cb_img, 'Parent', f);
            end

            % Plot extrinsics --------------------------------------------%

            debug.plot_multi_extrinsics({calib.cam(1).extrin.R}, ...
                                        {calib.cam(1).extrin.t}, ...
                                        {calib.cam.R_1}, ...
                                        {calib.cam.t_1}, ...
                                        colors, ...
                                        alphas, ...
                                        calib.config, ...
                                        axes_extrinsics);
            title(axes_extrinsics, 'Extrinsics', 'FontSize', 10);
            drawnow

            % Plot residuals ---------------------------------------------%

            for i = 1:num_cams %#ok<FXUP>
                debug.plot_res(ress(:, i), colors, alphas, max_res, axes_ress(i));
                title(axes_ress(i), ['Residuals (' num2str(i) ')'], 'FontSize', 10);
                xlabel(axes_ress(i), {['Board mean: [' num2str(mean(ress{idx_board, i})) ']'], ...
                                      ['Board stddev: [' num2str(std(ress{idx_board, i})) ']'], ...
                                      ['Overall mean: [' num2str(mean(vertcat(ress{:, i}))) ']'], ...
                                      ['Overall stddev: [' num2str(std(vertcat(ress{:, i}))) ']']}, ...
                       'FontSize', 7);
                drawnow
            end

            % Plot calibration board class -------------------------------%

            debug.plot_cb_geom(calib.config.obj_cb_geom, axes_cb_geom);
            title(axes_cb_geom, 'Calibration board geometry', 'FontSize', 10);
            drawnow

            % Plot calibrated board image --------------------------------%

            for i = 1:num_cams %#ok<FXUP>
                debug.plot_calib_cb_img(calib.cam(i).extrin(idx_board), ...
                                        calib.cam(i).intrin, ...
                                        axes_calib_cb_imgs(i));
                title(axes_calib_cb_imgs(i), ['Board (' num2str(i) ')'], ...
                      'FontSize', 10, 'Interpreter', 'none');
                xlabel(axes_calib_cb_imgs(i), {['Name: ' calib.cam(i).extrin(idx_board).img_cb.get_name()], ...
                                               ['Size: [' num2str(calib.cam(i).extrin(idx_board).img_cb.get_size()) ']']}, ...
                       'FontSize', 8, 'Interpreter', 'none');
                drawnow
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
            set(f, 'Name', ['Board: ' num2str(idx_board) ' of ' num2str(num_boards) '; mode: ' mode '; (NOTE: press left, right, "w", and "esc" key arrows to toggle)']);

            % Set bounding box
            for i = 1:num_cams %#ok<FXUP>
                switch mode
                    case 'whole'
                        bb = bb_img(calib.cam(i).extrin(idx_board).img_cb);
                    case 'worst'
                        bb = bb_worst(ress{idx_board, i}, calib.cam(i).extrin(idx_board).p_cb_p_ds(calib.cam(i).extrin(idx_board).idx_valid, :));
                end

                set(axes_calib_cb_imgs(i), 'Xlim', bb(:, 1), 'Ylim', bb(:, 2));
            end
        catch e
            if ishandle(f)
                rethrow(e);
            end
        end
    end
end

function bb = bb_img(img_cb)
    % Get size of image
    size_img = img_cb.get_size();

    % Set bounding box
    bb = [0.5 0.5; ...
          size_img(2)+0.5 size_img(1)+0.5];
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

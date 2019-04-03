function calib = calib_fp(img_cbs, p_fp_p_dss, calib_config, intrins)
    % Performs four point calibration.
    %
    % Inputs:
    %   img_cbs - class.img.intf; MxN calibration board image interfaces.
    %   p_fp_p_dss - cell; MxN cell of four point boxes around the
    %       calibration board images in distorted pixel coordinates
    %   calib_config - struct; struct returned by intf.load_calib_config()
    %   intrins - struct; optional. If passed in, intrinsics will not be
    %       optimized.
    %       .A - array; camera matrix
    %       .d - array; array of distortion coefficients
    %
    % Outputs:
    %   calib - struct;
    %       .config - struct; copy of input calib_config
    %       .cam - struct; calibration for i'th camera

    % Get single homography calibration function
    switch calib_config.calib_optimization
        case 'distortion_refinement'
            f_single_calib_H = @alg.single_calib_H_dr;
        otherwise
            error(['Unknown calibration optimization: "' calib_config.calib_optimization '"']);
    end

    % Get world to pixel stuff
    switch calib_config.calib_optimization
        case 'distortion_refinement'
            switch calib_config.target
                case 'checker'
                    obj_cb_w2p = class.calib.cb_w2p_p2p(calib_config);
                case 'circle'
                    obj_cb_w2p = class.calib.cb_w2p_c2e(calib_config);
                otherwise
                    error(['Cannot obtain world to pixel parameterization ' ...
                           'for ' calib_config.calib_optimization ' ' ...
                           'optimization with ' calib_config.target ' ' ...
                           'target.']);
            end
        otherwise
            error(['Cannot obtain world to pixel parameterization for ' ...
                   calib_config.calib_optimization ' optimization.']);
    end

    % Get distortion object
    obj_distortion = class.distortion.base(calib_config.sym_p_p2p_p_d, calib_config);

    % Get calibration object
    obj_calib = class.calib.base(calib_config.obj_A, ...
                                 calib_config.obj_R, ...
                                 obj_cb_w2p, ...
                                 obj_distortion, ...
                                 calib_config);

    % Get initial guess for homographies ---------------------------------%

    % Get number of cameras and boards
    num_cams = size(img_cbs, 2);
    num_boards = size(img_cbs, 1);

    % Get the four point boxes in world coordinates
    p_fp_ws = calib_config.obj_cb_geom.get_p_fp_ws();

    H_w2ps = cell(num_boards, num_cams);
    for i = 1:num_cams
        for j = 1:num_boards
            % Get four point box in pixel coordinates
            if exist('intrins', 'var')
                % If intrinsics are passed in, undistort four point box
                p_fp_ps = obj_distortion.p_p_d2p_p(p_fp_p_dss{j, i}, ...
                                                   p_fp_p_dss{j, i}, ...     % Use distorted points for initial guess
                                                   intrins(i).A, ...
                                                   intrins(i).d);
            else
                % If intrinsics arent available, assume distortion is small
                p_fp_ps = p_fp_p_dss{j, i};
            end

            % Compute homography - use direct p2p method
            H_w2ps{j, i} = alg.homography_p2p(p_fp_ws, ...
                                              p_fp_ps, ...
                                              calib_config);
        end
    end

    % Call multi homography calibration function -------------------------%

    if exist('intrins', 'var')
        calib = alg.multi_calib_H(f_single_calib_H, ...
                                  obj_calib, ...
                                  calib_config.obj_cb_geom, ...
                                  img_cbs, ...
                                  H_w2ps, ...
                                  calib_config, ...
                                  intrins);
    else
        calib = alg.multi_calib_H(f_single_calib_H, ...
                                  obj_calib, ...
                                  calib_config.obj_cb_geom, ...
                                  img_cbs, ...
                                  H_w2ps, ...
                                  calib_config);
    end

    % Append four point box to output ------------------------------------%

    for i = 1:num_cams
        for j = 1:num_boards
            calib.cam(i).extrin(j).p_fp_p_ds = p_fp_p_dss{j, i};
        end
    end
end

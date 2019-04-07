classdef custom < class.cb_geom.size_intf & class.cb_geom.fp_intf & class.cb_geom.target_intf
    % This is an example custom calibration board geometry

    methods(Access = public)
        function obj = custom(opts) %#ok<INUSD>
        end

        % Abstract methods -----------------------------------------------%

        function h = get_cb_height(obj) %#ok<MANU>
            h = 1000;
        end

        function w = get_cb_width(obj) %#ok<MANU>
            w = 1000;
        end

        function p_fp_ws = get_p_fp_ws(obj) %#ok<MANU>
            p_fp_ws = [275 275;
                       300 650;
                       700 400;
                       600 700];
        end

        function p_cb_ws = get_p_cb_ws(obj) %#ok<MANU>
            p_cb_ws = [125 450;
                       375 825;
                       500 500;
                       675 125;
                       825 625];

        end

        function boundary_ws = get_p_cb_w_boundaries(obj)
            p_cb_ws = obj.get_p_cb_ws();

            boundary_ws = cell(size(p_cb_ws, 1), 1);
            for i = 1:size(p_cb_ws, 1)
                boundary_ws{i} = [p_cb_ws(i, 1)-112.5 p_cb_ws(i, 2); ...
                                  p_cb_ws(i, 1) p_cb_ws(i, 2)+112.5; ...
                                  p_cb_ws(i, 1)+112.5 p_cb_ws(i, 2); ...
                                  p_cb_ws(i, 1) p_cb_ws(i, 2)-112.5];
            end
        end
    end
end

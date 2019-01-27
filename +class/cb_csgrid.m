classdef cb_csgrid < class.cb_pattern
    % This is the class definition for a centered square grid calibration
    % board pattern.

    properties(Access = private)
        num_targets_height      % int
        num_targets_width       % int
        target_spacing          % float
        idx_target_removal      % int array 
    end

    methods(Access = public)
        function obj = cb_csgrid(opts)
            obj@class.cb_pattern(opts);

            obj.num_targets_height = opts.num_targets_height;
            obj.num_targets_width = opts.num_targets_width;
            obj.target_spacing = opts.target_spacing;
            if isfield(opts, 'idx_target_removal')
                obj.idx_target_removal = opts.idx_target_removal;
            end
        end

        function num = get_num_targets_height(obj)
            num = obj.num_targets_height;
        end

        function num = get_num_targets_width(obj)
            num = obj.num_targets_width;
        end

        function spacing = get_target_spacing(obj)
            spacing = obj.target_spacing;
        end

        function mat = get_idx_target_removal(obj)
            mat = obj.idx_target_removal;
        end

        % Abstract methods -----------------------------------------------%

        function p_cb_ws = get_p_cb_ws(obj)
            % Get half height and half width of calibration board
            hh_cb = obj.get_cb_height()/2;
            hw_cb = obj.get_cb_width()/2;

            % Get half height and half width of grid
            hh_g = (obj.get_num_targets_height()-1)*obj.get_target_spacing()/2;
            hw_g = (obj.get_num_targets_width()-1)*obj.get_target_spacing()/2;

            % Get board points
            [y_cb_ws, x_cb_ws] = ndgrid(linspace(hh_cb-hh_g, hh_cb+hh_g, obj.get_num_targets_height()), ...
                                        linspace(hw_cb-hw_g, hw_cb+hw_g, obj.get_num_targets_width()));
            p_cb_ws = [x_cb_ws(:) y_cb_ws(:)];

            % Apply idx_target_removal
            p_cb_ws(reshape(obj.get_idx_target_removal(), [], 1), :) = [];
        end

        function boundary_ws = get_p_cb_w_boundaries(obj)
            % Get calibration board world points
            p_cb_ws = obj.get_p_cb_ws();

            % Get half spacing
            hs = obj.get_target_spacing()/2;

            % Iterate
            for i = 1:size(p_cb_ws, 1)
                % Get boundary around this point
                boundary_ws{i} = [p_cb_ws(i, 1)-hs p_cb_ws(i, 2)-hs; ...
                                  p_cb_ws(i, 1)-hs p_cb_ws(i, 2)+hs; ...
                                  p_cb_ws(i, 1)+hs p_cb_ws(i, 2)+hs; ...
                                  p_cb_ws(i, 1)+hs p_cb_ws(i, 2)-hs]; %#ok<AGROW>
            end
        end
    end
end

classdef cdgrid < class.cb_geom.target
    % This is the class definition for a centered diamond grid calibration
    % board geometry.

    properties(Access = private)
        num_targets_height      % int
        num_targets_width       % int
        target_spacing          % float
        first_on                % logical
        idx_target_removal      % int array
    end

    methods(Access = protected)    
        function num = get_num_targets_height(obj)
            num = obj.num_targets_height;
        end

        function num = get_num_targets_width(obj)
            num = obj.num_targets_width;
        end

        function spacing = get_target_spacing(obj)
            spacing = obj.target_spacing;
        end

        function fo = get_first_on(obj)
            fo = obj.first_on;
        end

        function mat = get_idx_target_removal(obj)
            mat = obj.idx_target_removal;
        end
    end
    
    methods(Access = public)
        function obj = cdgrid(opts)
            obj@class.cb_geom.target(opts);

            obj.num_targets_height = opts.num_targets_height;
            obj.num_targets_width = opts.num_targets_width;
            obj.target_spacing = opts.target_spacing;
            obj.first_on = opts.first_on;
            if isfield(opts, 'idx_target_removal')
                obj.idx_target_removal = opts.idx_target_removal;
            end
        end

        % Abstract methods -----------------------------------------------%

        function p_cb_ws = get_p_cb_ws(obj)
            % Get half height and half width of calibration board
            hh_cb = obj.get_cb_height()/2;
            hw_cb = obj.get_cb_width()/2;

            % Get half height and half width of grid
            hh_g = (obj.get_num_targets_height()-1)*obj.get_target_spacing()/2;
            hw_g = (obj.get_num_targets_width()-1)*obj.get_target_spacing()/2;

            % Get grid spacing
            spacing_x = linspace(hw_cb-hw_g, hw_cb+hw_g, obj.get_num_targets_width());
            spacing_y = linspace(hh_cb-hh_g, hh_cb+hh_g, obj.get_num_targets_height());

            % Get board points
            fo = obj.get_first_on();
            p_cb_ws = [];
            for i = 1:numel(spacing_x)
                if fo
                    y_cb_ws = spacing_y(1:2:end)';
                    fo = false;
                else
                    y_cb_ws = spacing_y(2:2:end)';
                    fo = true;
                end

                p_cb_ws = vertcat(p_cb_ws, [repmat(spacing_x(i), numel(y_cb_ws), 1) y_cb_ws]); %#ok<AGROW>
            end

            % Apply idx_target_removal
            p_cb_ws(reshape(obj.get_idx_target_removal(), [], 1), :) = [];
        end

        function boundary_ws = get_p_cb_w_boundaries(obj)
            % Get calibration board world points
            p_cb_ws = obj.get_p_cb_ws();

            % Get spacing
            s = obj.get_target_spacing();

            % Iterate
            for i = 1:size(p_cb_ws, 1)
                % Get boundary around this point
                boundary_ws{i} = [p_cb_ws(i, 1)-s p_cb_ws(i, 2); ...
                                  p_cb_ws(i, 1)   p_cb_ws(i, 2)+s; ...
                                  p_cb_ws(i, 1)+s p_cb_ws(i, 2); ...
                                  p_cb_ws(i, 1)   p_cb_ws(i, 2)-s]; %#ok<AGROW>
            end
        end
    end
end

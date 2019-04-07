classdef base < class.distortion.intf %#ok<*PROPLC>
    % This is the base class definition for camera distortion.

    properties(Access = private)
        sym_p_p2p_p_d
        f_p_p2p_p_d
        f_dp_p_d_dargs
        opts
    end

    methods(Static, Access = public)
        function validate_sym_p_p2p_p_d(sym_p_p2p_p_d)
            %   f(x_p, y_p, a_x, a_y, s, x_o, y_o, d_1, ..., d_M)
            args = arrayfun(@char, argnames(sym_p_p2p_p_d), 'UniformOutput', false);
            if ~all(strcmp(args(1:7), {'x_p', 'y_p', 'a_x', 'a_y', 's', 'x_o', 'y_o'}))
                error('Symbolic distortion function must have arguments which start with: (x_p, y_p, a_x, a_y, s, x_o, y_o)');
            end
        end
    end

    methods(Static, Access = private)
        function [a_x, a_y, s, x_o, y_o] = parse_A(A)
            % A = [a_x,   s, x_o;
            %        0, a_y, y_o;
            %        0,   0,   1];

            % Check to make sure this is a valid camera matrix
            if any(A([2 3 6]) ~= 0) || A(9) ~= 1
                error('Invalid camera matrix.');
            end

            % Parse camera matrix
            a_x = A(1, 1);
            a_y = A(2, 2);
            s = A(1, 2);
            x_o = A(1, 3);
            y_o = A(2, 3);
        end

        function out = f_distortion(f, p_ps, A, d)
            % Parse A and d
            [a_x, a_y, s, x_o, y_o] = class.distortion.base.parse_A(A);
            d_cell = num2cell(d);

            % Call distortion function
            out = f(p_ps(:, 1), ...
                    p_ps(:, 2), ...
                    a_x, ...
                    a_y, ...
                    s, ...
                    x_o, ...
                    y_o, ...
                    d_cell{:});

            % If output is a constant, sometimes the output is a single
            % value; if this is the case, repmat until the size is equal to
            % the size of p_ps.
            if ~isequal(size(out), size(p_ps))
                out = repmat(out, size(p_ps, 1), 1);
            end
        end

        function jacob = dp_p_d_darg(f, p_ps, A, d)
            % Compute partial derivatives
            jacob = class.distortion.base.f_distortion(f, p_ps, A, d);

            % Reshape so jacobian is in the desired output format
            jacob = reshape(jacob', [], 1);
        end
    end

    methods(Access = private)
        function sym_p_p2p_p_d = get_sym_p_p2p_p_d(obj)
            sym_p_p2p_p_d = obj.sym_p_p2p_p_d;
        end

        function f_p_p2p_p_d = get_f_p_p2p_p_d(obj)
            f_p_p2p_p_d = obj.f_p_p2p_p_d;
        end

        function f_dp_p_d_dargs = get_f_dp_p_d_dargs(obj)
            f_dp_p_d_dargs = obj.f_dp_p_d_dargs;
        end

        function opts = get_opts(obj)
            opts = obj.opts;
        end
    end

    methods(Access = public)
        function obj = base(sym_p_p2p_p_d, opts)
            % Get function handle
            f_p_p2p_p_d = matlabFunction(sym_p_p2p_p_d);

            % Get function handles for distortion function partial derivatives
            args_p_p2p_p_d = argnames(sym_p_p2p_p_d);            
            f_dp_p_d_dargs = cell(numel(args_p_p2p_p_d), 1);
            for i = 1:numel(args_p_p2p_p_d)
                % Differentiate
                f_dp_p_d_dargs{i} = diff(sym_p_p2p_p_d, args_p_p2p_p_d(i));

                % Convert to function handle
                f_dp_p_d_dargs{i} = matlabFunction(f_dp_p_d_dargs{i});
            end

            % Store stuff
            obj.sym_p_p2p_p_d = sym_p_p2p_p_d;
            obj.f_p_p2p_p_d = f_p_p2p_p_d;
            obj.f_dp_p_d_dargs = f_dp_p_d_dargs;
            obj.opts = opts;
        end

        function num_params_d = get_num_params_d(obj)
            % Distortion function has format:
            %   f(x_p, y_p, a_x, a_y, s, x_o, y_o, d_1, ..., d_M)
            %
            % So number of distortion params is num_args - 7

            % Get number of arguments
            num_args = nargin(obj.get_f_p_p2p_p_d());

            % Get number of distortion parameters
            num_params_d = num_args - 7;
        end

        function args = get_d_args(obj)                        
            num_params_d = obj.get_num_params_d();
            
            sym_args = argnames(obj.get_sym_p_p2p_p_d());
            args = cell(num_params_d, 1);
            for i = 1:num_params_d
                args{i} = char(sym_args(i+7));
            end
        end

        function jacob = dp_p_d_dp_p(obj, p_ps, A, d)
            %   Format of jacobian is:
            %
            %             dx_p_1 dy_p_1 ... dx_p_N dy_p_N
            %   dx_p_d_1
            %   dy_p_d_1
            %       .
            %       .
            %       .
            %   dx_p_d_N
            %   dy_p_d_N

            f_dp_p_d_dargs = obj.get_f_dp_p_d_dargs();

            % Compute partial derivatives
            dp_p_d_dx_p = class.distortion.base.dp_p_d_darg(f_dp_p_d_dargs{1}, p_ps, A, d); % 1st index is x_p
            dp_p_d_dy_p = class.distortion.base.dp_p_d_darg(f_dp_p_d_dargs{2}, p_ps, A, d); % 2nd index is y_p

            % TODO: find a better way to construct sparse diagonal jacobian,
            % as this is probably a bottleneck

            % Jacobian
            jacob_cell = mat2cell(sparse([dp_p_d_dx_p dp_p_d_dy_p]), 2*ones(1, size(p_ps, 1)), 2);
            jacob = blkdiag(jacob_cell{:});
        end

        function jacob = dp_p_d_dA(obj, p_ps, A, d)
            %   Format of jacobian is:
            %
            %            dA_11 dA_21 dA_31 dA_12 dA_22 dA_32 dA_13 dA_23 dA_33
            %   dx_p_d_1
            %   dy_p_d_1
            %      .
            %      .
            %      .
            %   dx_p_d_N
            %   dy_p_d_N

            % NOTE:
            %   A = [A_11, A_12, A_13; = [a_x,   s, x_o;
            %        A_21, A_22, A_23;      0, a_y, y_o;
            %        A_31, A_32, A_33]      0,   0,   1]

            f_dp_p_d_dargs = obj.get_f_dp_p_d_dargs();

            dp_p_d_da_x = class.distortion.base.dp_p_d_darg(f_dp_p_d_dargs{3}, p_ps, A, d); % 3rd index is a_x
            dp_p_d_da_y = class.distortion.base.dp_p_d_darg(f_dp_p_d_dargs{4}, p_ps, A, d); % 4th index is a_y
            dp_p_d_ds   = class.distortion.base.dp_p_d_darg(f_dp_p_d_dargs{5}, p_ps, A, d); % 5th index is s
            dp_p_d_dx_o = class.distortion.base.dp_p_d_darg(f_dp_p_d_dargs{6}, p_ps, A, d); % 6th index is x_o
            dp_p_d_dy_o = class.distortion.base.dp_p_d_darg(f_dp_p_d_dargs{7}, p_ps, A, d); % 7th index is y_o

            jacob = [dp_p_d_da_x, ...                   % A_11
                     zeros(2*size(p_ps, 1), 1), ...     % A_21
                     zeros(2*size(p_ps, 1), 1), ...     % A_31
                     dp_p_d_ds, ...                     % A_12
                     dp_p_d_da_y, ...                   % A_22
                     zeros(2*size(p_ps, 1), 1), ...     % A_32
                     dp_p_d_dx_o, ...                   % A_13
                     dp_p_d_dy_o, ...                   % A_23
                     zeros(2*size(p_ps, 1), 1)];        % A_33
        end

        function jacob = dp_p_d_dd(obj, p_ps, A, d)
            %   Format of jacobian is:
            %
            %            dd_1 ... dd_M
            %   dx_p_d_1
            %   dy_p_d_1
            %      .
            %      .
            %      .
            %   dx_p_d_N
            %   dy_p_d_N

            f_dp_p_d_dargs = obj.get_f_dp_p_d_dargs();

            num_params_d = obj.get_num_params_d();
            
            jacob = zeros(2*size(p_ps, 1), num_params_d);
            for i = 1:num_params_d
                dp_p_d_dd_i = class.distortion.base.dp_p_d_darg(f_dp_p_d_dargs{7+i}, p_ps, A, d); % Offset of 7
                jacob(:, i) = dp_p_d_dd_i;
            end
        end

        function p_p_ds = p_p2p_p_d(obj, p_ps, A, d)
            f_p_p2p_p_d = obj.get_f_p_p2p_p_d();

            % Apply transform to coordinates
            p_p_ds = class.distortion.base.f_distortion(f_p_p2p_p_d, p_ps, A, d);
        end

        function p_ps = p_p_d2p_p(obj, p_p_ds, p_ps_init, A, d)
            % Get opts
            opts = obj.get_opts();

            % Initialize parameter vector
            params = reshape(p_ps_init', [], 1);
            for it = 1:opts.p_p_d2p_p_it_cutoff
                % Get p_ps from params
                p_ps = reshape(params, 2, [])';

                % Jacobian
                jacob = obj.dp_p_d_dp_p(p_ps, A, d);

                % Residual
                res = reshape(obj.p_p2p_p_d(p_ps, A, d)', [], 1) - reshape(p_p_ds', [], 1);

                % Get and store update
                delta_params = -alg.safe_lscov(jacob, res);
                params = params + delta_params;

                % Exit if change in distance is small
                if norm(delta_params) < opts.p_p_d2p_p_norm_cutoff
                    break
                end
            end

            % Get p_p
            p_ps = reshape(params, 2, [])';
        end
    end
end

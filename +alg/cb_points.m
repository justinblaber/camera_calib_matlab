function [board_points, four_points] = cb_points(cb_config)
    % This will return the world coordinate points of the calibration
    % board and the world coordinates of the four point box around the
    % board.
    %
    % Inputs:
    %   cb_config - struct; this is the struct returned by
    %       util.load_cb_config()
    %
    % Outputs:
    %   board_points - array; Nx2 array of points of calibration board in 
    %       world coordinates
    %   four_points - array; 4x2 array of four points surrounding the 
    %       calibration board used for initial guess of checkerboard 
    %       corners
    
    % Get four points first
    four_points = [0                            0;
                   0                            cb_config.four_point_height;
                   cb_config.four_point_width   0;
                   cb_config.four_point_width   cb_config.four_point_height];
    
    % Get board points next
    board_height = cb_config.num_squares_height * cb_config.square_size;
    board_width = cb_config.num_squares_width * cb_config.square_size;           
    [board_y, board_x] = ndgrid(0:cb_config.square_size:board_height, ...
                                0:cb_config.square_size:board_width);
    
    % Assume board is centered between four points 
    board_points = [board_x(:)-board_width/2+cb_config.four_point_width/2 ...
                    board_y(:)-board_height/2+cb_config.four_point_height/2];
end
function [board_points_w, four_points_w] = cb_points(cal_config)
    % This will return the calibration board points in world coordinates
    % and the four point box around the board in world coordinates.
    %
    % Inputs:
    %   cal_config - struct; this is the struct returned by
    %       util.load_cal_config()
    %
    % Outputs:
    %   board_points_w - array; Nx2 array of calibration board points in 
    %       world coordinates
    %   four_points_w - array; 4x2 array of four point box around the board
    %       in world coordinates.
    
    % Get four points first
    four_points_w = [0                             0;
                     0                             cal_config.four_point_height;
                     cal_config.four_point_width   0;
                     cal_config.four_point_width   cal_config.four_point_height];
    
    % Get board points next
    board_height = cal_config.num_squares_height * cal_config.square_size;
    board_width = cal_config.num_squares_width * cal_config.square_size;           
    [board_y, board_x] = ndgrid(0:cal_config.square_size:board_height, ...
                                0:cal_config.square_size:board_width);
    
    % Assume board is centered between four points 
    board_points_w = [board_x(:)-board_width/2+cal_config.four_point_width/2 ...
                      board_y(:)-board_height/2+cal_config.four_point_height/2];
end

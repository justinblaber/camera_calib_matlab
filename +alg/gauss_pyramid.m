function pyramid = gauss_pyramid(array,calib_config)
    % Constructs image pyramid described in David Lowe's SIFT paper.     
    % 
    % Inputs:
    %   array - array; MxN array
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   pyramid - cell; cell has length equal to num_octaves. Each
    %       successive octave is scaled by half. The number of images in
    %       each octave is equal to s+3.    
    
    % Set k - gaussians are separated by this constant factor in scale
    % space.
    k = 2^(1/calib_config.blob_detect_s);

    % Create pyramid
    pyramid = {};
    for i = 1:calib_config.blob_detect_num_octaves % Cycle over octaves                
        for j = 1:calib_config.blob_detect_s+3 % Cycle over scales
            if i == 1 && j == 1
                % First scale of first octave; initialize
                pyramid{i} = alg.array_gauss(array,k^-1); %#ok<AGROW>
            elseif i > 1 && j == 1
                % This is the first scale after the first octave. Take the 
                % 2nd from the top image and downsample by 2
                pyramid{i} = pyramid{i-1}(1:2:end,1:2:end,end-2); %#ok<AGROW>
            else
                % These are scales after the first, sigma must be:
                sigma = sqrt(k^(2*(j-2))-k^(2*(j-2)-2));
                pyramid{i} = cat(3,pyramid{i},alg.array_gauss(pyramid{i}(:,:,j-1),sigma)); %#ok<AGROW>
            end         
        end
    end
end
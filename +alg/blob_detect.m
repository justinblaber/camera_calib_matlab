function blobs = blob_detect(array,calib_config)
    % Performs blob detection. By default this returns dark blobs. If you
    % want both dark and light blobs you can process the array and the 
    % negative of the array and then merge the results.
    % 
    % Inputs:
    %   array - array; MxN array
    %   calib_config - struct; this is the struct returned by
    %       util.read_calib_config()
    %
    % Outputs:
    %   blobs - struct; contains:
    %       .x - scalar; x location of blob
    %       .y - scalar; y location of blob
    %       .sigma - scalar; "scale" of blob
    %       .M - array; second moment matrix at location of blob at
    %           specified scale in scale space
    
    % array must have intensities between 0 and 1 for the dog intensity
    % filtering to work
    array = (array-min(array(:)))./(max(array(:))-min(array(:)));
    
    % Set k - gaussians are separated by this constant factor in scale
    % space.
    k = 2^(1/calib_config.blob_detect_s);
    
    % Get gaussian pyramid
    gauss_pyramid = alg.gauss_pyramid(array,calib_config);
    
    % Compute difference of gaussian
    dog_pyramid = {};
    for i = 1:length(gauss_pyramid)
        dog_pyramid{i} = gauss_pyramid{i}(:,:,2:end) - gauss_pyramid{i}(:,:,1:end-1); %#ok<AGROW>
    end
    
    % Get gradient of gaussian pyramid - used to compute second moment
    % matrix for affine invariance
    grad_pyramid.x = {};
    grad_pyramid.y = {};
    for i = 1:length(gauss_pyramid)
        grad_pyramid.x{i} = [];
        grad_pyramid.y{i} = [];
        for j = 1:size(gauss_pyramid{i},3)
            grad_pyramid.x{i} = cat(3,grad_pyramid.x{i},alg.array_grad(gauss_pyramid{i}(:,:,j),'x'));
            grad_pyramid.y{i} = cat(3,grad_pyramid.y{i},alg.array_grad(gauss_pyramid{i}(:,:,j),'y'));
        end
    end
        
    % Get blobs
    blobs = struct('x',{},'y',{},'sigma',{},'M',{});
    for i = 1:length(dog_pyramid)
        for j = 2:size(dog_pyramid{i},3)-1 % Skip first and last scales
            % Assign, to every voxel, the maximum of its neighbors. Then
            % see if voxel value is greater than this value; if this is 
            % true, then it's a local maxima (technique is from Jonas on 
            % stackoverflow). Note: finding maxima will return DARK blobs.
                        
            kernel = true(3,3,3);
            kernel(2,2,2) = false;
            dog_dilate = imdilate(dog_pyramid{i}(:,:,j-1:j+1),kernel);
            maxima_pyr = dog_pyramid{i}(:,:,j) > dog_dilate(:,:,2);

            % Clear out edge values
            maxima_pyr(1,:) = false;
            maxima_pyr(end,:) = false;
            maxima_pyr(:,1) = false;
            maxima_pyr(:,end) = false;
            
            % Get "pyramid coordinates" of maxima. Use "pyramid 
            % coordinates" to distinguish these from pixel coordinates wrt
            % the original image.
            [y_maxima_pyr,x_maxima_pyr] = find(maxima_pyr);

            % Compute sigmas and change in sigmas to get finite
            % difference approximation of derivatives 
            octave_factor = (2^(i-1));
            sigma = (k^(j-2))*octave_factor;
            dsigma_1 = sigma-(k^(j-3))*octave_factor; 
            dsigma_2 = (k^(j-1))*octave_factor-sigma;
            
            % Go through points and do Lowe's cascade
            for m = 1:length(y_maxima_pyr)    
                % Refine location and scale first
                
                % Calculate gradient: [ddog/dx_pyr ddog/dy_pyr ddog/dsigma]
                % where p = [x_pyr y_pyr sigma]
                dg_dp(1) = (dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)+1,j)-dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)-1,j))/2;
                dg_dp(2) = (dog_pyramid{i}(y_maxima_pyr(m)+1,x_maxima_pyr(m),j)-dog_pyramid{i}(y_maxima_pyr(m)-1,x_maxima_pyr(m),j))/2; 
                % Take average of both derivatives for sigma
                dg_dp(3) = ((dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j+1)-dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j))/dsigma_2 + ...
                            (dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j)-dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j-1))/dsigma_1)/2;

                % Calculate hessian:
                %   [d^2dog/dx_pyr^2        d^2dog/(dx_pyr*dy_pyr) d^2dog/(dx_pyr*dsigma)
                %    d^2dog/(dy_pyr*dx_pyr) d^2dog/dy_pyr^2        d^2dog/(dy_pyr*dsigma)
                %    d^2dog/(dsigma*dx_pyr) d^2dog/(dsigma*dy_pyr) d^2dog/(dsigma^2)]
                ddg_ddp(1,1) = dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)+1,j) - ...
                               2*dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j) + ...
                               dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)-1,j);
                ddg_ddp(1,2) = ((dog_pyramid{i}(y_maxima_pyr(m)+1,x_maxima_pyr(m)+1,j)-dog_pyramid{i}(y_maxima_pyr(m)-1,x_maxima_pyr(m)+1,j))/2 - ...
                                (dog_pyramid{i}(y_maxima_pyr(m)+1,x_maxima_pyr(m)-1,j)-dog_pyramid{i}(y_maxima_pyr(m)-1,x_maxima_pyr(m)-1,j))/2)/2;
                ddg_ddp(1,3) = (((dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)+1,j+1)-dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)+1,j))/dsigma_2 + ...
                                 (dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)+1,j)-dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)+1,j-1))/dsigma_1)/2 - ...
                                ((dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)-1,j+1)-dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)-1,j))/dsigma_2 + ...
                                 (dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)-1,j)-dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m)-1,j-1))/dsigma_1)/2)/2;
                ddg_ddp(2,2) = dog_pyramid{i}(y_maxima_pyr(m)+1,x_maxima_pyr(m),j) - ...
                               2*dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j) + ...
                               dog_pyramid{i}(y_maxima_pyr(m)-1,x_maxima_pyr(m),j); 
                ddg_ddp(2,3) = (((dog_pyramid{i}(y_maxima_pyr(m)+1,x_maxima_pyr(m),j+1)-dog_pyramid{i}(y_maxima_pyr(m)+1,x_maxima_pyr(m),j))/dsigma_2 + ...
                                 (dog_pyramid{i}(y_maxima_pyr(m)+1,x_maxima_pyr(m),j)-dog_pyramid{i}(y_maxima_pyr(m)+1,x_maxima_pyr(m),j-1))/dsigma_1)/2 - ...
                                ((dog_pyramid{i}(y_maxima_pyr(m)-1,x_maxima_pyr(m),j+1)-dog_pyramid{i}(y_maxima_pyr(m)-1,x_maxima_pyr(m),j))/dsigma_2 + ...
                                 (dog_pyramid{i}(y_maxima_pyr(m)-1,x_maxima_pyr(m),j)-dog_pyramid{i}(y_maxima_pyr(m)-1,x_maxima_pyr(m),j-1))/dsigma_1)/2)/2; 
                ddg_ddp(3,3) = ((dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j+1)-dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j))/dsigma_2 - ...
                                (dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j)-dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j-1))/dsigma_1)/((dsigma_2+dsigma_1)/2);

                % Fill lower half of hessian
                ddg_ddp(2,1) = ddg_ddp(1,2);
                ddg_ddp(3,1) = ddg_ddp(1,3);
                ddg_ddp(3,2) = ddg_ddp(2,3);  
                                
                % Find incremental parameters
                delta_p = -pinv(ddg_ddp)*dg_dp';   

                % Get optimized locations
                x_maxima_opt_pyr = x_maxima_pyr(m)+delta_p(1);
                y_maxima_opt_pyr = y_maxima_pyr(m)+delta_p(2);
                sigma_opt = sigma+delta_p(3);  
                               
                % Filter out points
                if(abs(delta_p(1)) < 1 && abs(delta_p(2)) < 1 && abs(delta_p(3)) < dsigma_2 && ...                                                     % Points should not move that much
                   dog_pyramid{i}(y_maxima_pyr(m),x_maxima_pyr(m),j) + (1/2)*dg_dp*delta_p > calib_config.blob_detect_contrast_cutoff && ...           % Reject extrema with low contrast 
                   trace(ddg_ddp(1:2,1:2))^2/det(ddg_ddp(1:2,1:2)) < (calib_config.blob_detect_r_cutoff+1)^2/calib_config.blob_detect_r_cutoff && ...  % Eleminate edge response        
                   x_maxima_opt_pyr >= 1 && ...                                                                                                        % Make sure point is within the pyramid
                   x_maxima_opt_pyr <= size(dog_pyramid{i},2) && ...                                                                                   % ^
                   y_maxima_opt_pyr >= 1 && ...                                                                                                        % ^
                   y_maxima_opt_pyr <= size(dog_pyramid{i},1) && ...                                                                                   % ^
                   sigma_opt > 0)                                                                                                                      % sigma should be positive
                    % Compute second moment matrix for affine invariance.
                    % Interpolate gradients at center specified by 
                    % x_maxima_pyr and y_maxima_pyr. Do this at 
                    % non-optimized scale for now.
                    
                    % Get radius in pyramid coordinates
                    radius_pyr = (1+(k-1)/2)*sqrt(2)*sigma_opt/octave_factor;
                    
                    % Get gaussian kernel for weights
                    l_kernel = 4*ceil(radius_pyr)+1;
                    half_kernel = (l_kernel-1)/2;
                    kernel_gauss = reshape(fspecial('gaussian',[l_kernel l_kernel],radius_pyr),[],1);
                                        
                    % Coordinates
                    [y_pyr,x_pyr] = ndgrid(y_maxima_opt_pyr-half_kernel:y_maxima_opt_pyr+half_kernel, ...
                                           x_maxima_opt_pyr-half_kernel:x_maxima_opt_pyr+half_kernel);
                                       
                    % Interpolate gradients
                    grad_x = alg.array_interp(grad_pyramid.x{i}(:,:,j),[x_pyr(:) y_pyr(:)],'bicubic');
                    grad_y = alg.array_interp(grad_pyramid.y{i}(:,:,j),[x_pyr(:) y_pyr(:)],'bicubic');
                    
                    % Clear out out of bounds points
                    idx_out = x_pyr < 1 | x_pyr > size(dog_pyramid{i},2) | ...
                              y_pyr < 1 | y_pyr > size(dog_pyramid{i},1);
                    kernel_gauss(idx_out) = [];
                    grad_x(idx_out) = [];
                    grad_y(idx_out) = [];
                    
                    % Get second moment matrix
                    M(1,1) = sum(kernel_gauss(:).*grad_x.^2);
                    M(1,2) = sum(kernel_gauss(:).*grad_x.*grad_y);
                    M(2,2) = sum(kernel_gauss(:).*grad_y.^2);
                    M(2,1) = M(1,2);
                    
                    % Filter out flat blobs according to second moment
                    % matrix
                    D = eig(M);
                    if sqrt(max(D))/sqrt(min(D)) <= calib_config.blob_detect_second_moment_cutoff                 
                        % Store blobs
                        blobs(end+1).x = (x_maxima_opt_pyr-1)*octave_factor+1; %#ok<AGROW>
                        blobs(end).y = (y_maxima_opt_pyr-1)*octave_factor+1;
                        blobs(end).sigma = sigma_opt;
                        blobs(end).M = M;
                    end
                end
            end
        end
    end    
end
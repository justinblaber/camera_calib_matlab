function test_undistort_array
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    
    % Load distorted data
    load(fullfile(tests_path,'data','checker1_d.mat'));
    
    % Distort
    f_p_p2p_p_d = @(x_p,y_p,a,x_o,y_o,k1,k2,p1,p2)[x_o+a.*(p2.*(1.0./a.^2.*(x_o-x_p).^2.*3.0+1.0./a.^2.*(y_o-y_p).^2)-((x_o-x_p).*(k1.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2)+k2.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2).^2+1.0))./a+1.0./a.^2.*p1.*(x_o-x_p).*(y_o-y_p).*2.0),y_o+a.*(p1.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2.*3.0)-((y_o-y_p).*(k1.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2)+k2.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2).^2+1.0))./a+1.0./a.^2.*p2.*(x_o-x_p).*(y_o-y_p).*2.0)];
    a = [578.8932; 331.1893; 244.5372];
    d = [0.1738; 0.1930; 0.0025; 0.0080];    
    opts.undistort_array_interp = 'spline';
    array = alg.undistort_array(checker1_d,f_p_p2p_p_d,a,d,opts);
    
    % Assert
    abs_diff = abs(array - rgb2gray(im2double(imread(fullfile(tests_path,'data','checker1.jpg')))));
    abs_diff(isnan(abs_diff)) = 0;
    assert(all(all(abs_diff < 0.075)));
end
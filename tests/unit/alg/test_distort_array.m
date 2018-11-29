function test_distort_array
    % Get tests path
    tests_path = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    
    % Load image data
    array = rgb2gray(im2double(imread(fullfile(tests_path,'data','checker1.jpg'))));
    
    % Distort
    f_p_p2p_p_d = @(x_p,y_p,a,x_o,y_o,k1,k2,p1,p2)[x_o+a.*(p2.*(1.0./a.^2.*(x_o-x_p).^2.*3.0+1.0./a.^2.*(y_o-y_p).^2)-((x_o-x_p).*(k1.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2)+k2.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2).^2+1.0))./a+1.0./a.^2.*p1.*(x_o-x_p).*(y_o-y_p).*2.0),y_o+a.*(p1.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2.*3.0)-((y_o-y_p).*(k1.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2)+k2.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2).^2+1.0))./a+1.0./a.^2.*p2.*(x_o-x_p).*(y_o-y_p).*2.0)];
    f_dp_p_d_dx_p = @(x_p,y_p,a,x_o,y_o,k1,k2,p1,p2)[a.*((k1.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2)+k2.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2).^2+1.0)./a+((1.0./a.^2.*k1.*(x_o.*2.0-x_p.*2.0)+1.0./a.^2.*k2.*(x_o.*2.0-x_p.*2.0).*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2).*2.0).*(x_o-x_p))./a-1.0./a.^2.*p1.*(y_o-y_p).*2.0-1.0./a.^2.*p2.*(x_o.*2.0-x_p.*2.0).*3.0),-a.*(-((1.0./a.^2.*k1.*(x_o.*2.0-x_p.*2.0)+1.0./a.^2.*k2.*(x_o.*2.0-x_p.*2.0).*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2).*2.0).*(y_o-y_p))./a+1.0./a.^2.*p2.*(y_o-y_p).*2.0+1.0./a.^2.*p1.*(x_o.*2.0-x_p.*2.0))];
    f_dp_p_d_dy_p = @(x_p,y_p,a,x_o,y_o,k1,k2,p1,p2)[-a.*(-((1.0./a.^2.*k1.*(y_o.*2.0-y_p.*2.0)+1.0./a.^2.*k2.*(y_o.*2.0-y_p.*2.0).*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2).*2.0).*(x_o-x_p))./a+1.0./a.^2.*p1.*(x_o-x_p).*2.0+1.0./a.^2.*p2.*(y_o.*2.0-y_p.*2.0)),a.*((k1.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2)+k2.*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2).^2+1.0)./a+((1.0./a.^2.*k1.*(y_o.*2.0-y_p.*2.0)+1.0./a.^2.*k2.*(y_o.*2.0-y_p.*2.0).*(1.0./a.^2.*(x_o-x_p).^2+1.0./a.^2.*(y_o-y_p).^2).*2.0).*(y_o-y_p))./a-1.0./a.^2.*p2.*(x_o-x_p).*2.0-1.0./a.^2.*p1.*(y_o.*2.0-y_p.*2.0).*3.0)];
    a = [578.8932; 331.1893; 244.5372];
    d = [0.1738; 0.1930; 0.0025; 0.0080];    
    opts.p_p_d2p_p_it_cutoff = 20;
    opts.p_p_d2p_p_norm_cutoff = '1e-6';
    opts.distort_array_interp = 'spline';
    array_d = alg.distort_array(array,f_p_p2p_p_d,f_dp_p_d_dx_p,f_dp_p_d_dy_p,a,d,opts);
    
    % Assert    
    load(fullfile(tests_path,'data','checker1_d.mat'));
    assert(all(all(abs(checker1_d - array_d) < 1e-4)));
end
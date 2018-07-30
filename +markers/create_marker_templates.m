%% Clear
clear; clc;

%% Read markers and create templates

% Initialize figure
f = figure();

% Load marker config
markers_dir_path = fileparts(mfilename('fullpath'));
marker_config = util.read_data(fullfile(markers_dir_path,'marker.conf'));

% Get radius and theta samples
r_samples = linspace(marker_config.marker_radius*marker_config.radius_norm_range(1), ...
                     marker_config.marker_radius*marker_config.radius_norm_range(2), ...
                     marker_config.radius_num_samples);
% For theta, sample 1 more and then remove it; this allows [0 2*pi)
theta_samples = linspace(0,2*pi,marker_config.theta_num_samples+1);
theta_samples(end) = []; 

% Cycle over markers and create/save polar patch templates
marker_templates_path = fullfile(markers_dir_path,'marker_templates.txt');
fclose(fopen(marker_templates_path,'w')); % Creates/clears file
for i = 1:length(marker_config.marker_paths)
    % Read marker
    marker = imread(marker_config.marker_paths{i});
    marker = rgb2gray(im2double(marker));

    % Get coordinates
    x = bsxfun(@times,cos(theta_samples)',r_samples) + (size(marker,2)+1)/2; 
    y = bsxfun(@times,sin(theta_samples)',r_samples) + (size(marker,1)+1)/2; 
    
    % Get polar patch
    polar_patch = alg.array_interp(marker,[x(:) y(:)],'cubic'); 
    polar_patch = reshape(polar_patch,length(theta_samples),[]); 
    
    % Plot sampling points
    a = subplot(4,2,2*(i-1)+1,'parent',f);
    imshow(marker,[],'parent',a);
    hold(a,'on'); 
    plot(x(:),y(:),'-r','Parent',a); 
    hold(a,'off'); 
    
    % Plot polar patch
    a = subplot(4,2,2*i,'parent',f);
    imshow(polar_patch,[],'parent',a)
    
    % Write patch
    util.write_comment(['Marker ' num2str(i)],marker_templates_path);
    util.write_array(polar_patch,'polar_patches',marker_templates_path);
    util.write_newline(marker_templates_path);
end
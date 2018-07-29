%% Clear
clc; clear;

%% Run tests

disp('---'); 
disp(['Running unit tests...']); 
        
% Set directories in camera_calib to test
camera_calib_sub_dirs = {'+alg', ...
                         '+class', ...
                         '+util'};

% Keep track of passed, missing, and failed
passed = 0;
missing = 0;
failed = 0;

% Loop over directories and test
tests_dir_path = fileparts(mfilename('fullpath'));
camera_calib_path = fileparts(tests_dir_path);
camera_calib_l = dir_without_dots(camera_calib_path);
for i = 1:length(camera_calib_l)
    if camera_calib_l(i).isdir && ismember(camera_calib_l(i).name,camera_calib_sub_dirs)
        camera_calib_sub_dir_name = camera_calib_l(i).name;
        disp('---'); 
        disp(['Directory:    ' camera_calib_sub_dir_name '...']); 
                
        % Get function to test
        camera_calib_sub_dir_l = dir_without_dots(fullfile(camera_calib_path,camera_calib_sub_dir_name));
        for j = 1:length(camera_calib_sub_dir_l)
            if ~camera_calib_sub_dir_l(j).isdir
                func_name = camera_calib_sub_dir_l(j).name;
                
                % Get corresponding unit test; remove + from camera_calib_sub_dir_name
                unit_test_path = fullfile(tests_dir_path,'unit',camera_calib_sub_dir_name(2:end),['test_' func_name]);
                
                if exist(unit_test_path,'file') == 2
                    try
                        % Run test
                        run(unit_test_path);   
                        
                        % Test passed if it gets here
                        fprintf(['Passed:       ' func_name '... \n']);
                        passed = passed + 1;
                    catch e
                        % Test failed if it gets here
                        fprintf(2,['Failed:       ' func_name '... \n']);
                        failed = failed + 1;
                    end                    
                else
                    % Test is missing
                    fprintf(2,['Doesnt exist: ' func_name '... \n']);
                    missing = missing+1;
                end
            end
        end        
    end
end

% Print final message
disp('---');
msg = ['Test results: Passed - ' num2str(passed) '; Missing - ' num2str(missing) '; Failed - ' num2str(failed')];

% Return error if any tests are failed or missing; if you don't want to
% test a certain function for whatever reason, then just create an empty
% dummy test file.
if failed > 0 || missing > 0
    error(msg);
else
    disp(msg);
end    

function l = dir_without_dots(path)
    % Returns dir without the annoying dots
    l = dir(path);
    l = l(~ismember({l.name},{'.','..'}));
end
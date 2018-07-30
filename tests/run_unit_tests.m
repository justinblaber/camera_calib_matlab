%% Clear
clc; clear;

%% Run tests

disp('---'); 
disp('Running unit tests...'); 
        
% Set directories in camera_calib to test; This script will only pass if
% each function in the following folders has a corresponding unit test
% which contains the function it is supposed to be testing. Any additional
% unit tests found will result in an error of "unknown test(s)"
camera_calib_sub_dirs = {'+alg', ...
                         '+class', ...
                         '+util'};

% Keep track of passed, missing, and failed tests
passed = 0;
missing = 0;
failed = 0;

% Loop over directories and test
tests_dir_path = fileparts(mfilename('fullpath'));
camera_calib_path = fileparts(tests_dir_path);
l_camera_calib = dir_without_dots(camera_calib_path);
for i = 1:length(l_camera_calib)
    % Check to see if this folder is in camera_calib_sub_dirs
    if l_camera_calib(i).isdir && ismember(l_camera_calib(i).name,camera_calib_sub_dirs)
        camera_calib_sub_dir_name = l_camera_calib(i).name;
        disp('---'); 
        disp(['Directory:    ' camera_calib_sub_dir_name '...']); 
                        
        % Get unit test directory; remove + from camera_calib_sub_dir_name
        unit_test_dir_path = fullfile(tests_dir_path,'unit',camera_calib_sub_dir_name(2:end));
        
        % Get all unit tests in this directory
        l_unit_test_dir = dir_without_dots(unit_test_dir_path);
        unit_test_paths = fullfile(unit_test_dir_path,{l_unit_test_dir.name});
        
        % Get function to test
        l_camera_calib_sub_dir = dir_without_dots(fullfile(camera_calib_path,camera_calib_sub_dir_name));
        for j = 1:length(l_camera_calib_sub_dir)
            if l_camera_calib_sub_dir(j).isdir
                % There shouldn't be a directory
                unknown_dir_path = fullfile(l_camera_calib_sub_dir(j).folder,l_camera_calib_sub_dir(j).name);
                error(['Unknown directory: "' unknown_dir_path '"']);
            end
                        
            [~,func_name] = fileparts(l_camera_calib_sub_dir(j).name);

            % Get corresponding unit test; remove + from camera_calib_sub_dir_name
            unit_test_dir_name = camera_calib_sub_dir_name(2:end);
            unit_test_path = fullfile(tests_dir_path,'unit',unit_test_dir_name,['test_' func_name '.m']);

            if exist(unit_test_path,'file') == 2
                % Make sure test actually contains the function
                full_func_name = strjoin({unit_test_dir_name,func_name},'.');
                if ~contains(fileread(unit_test_path),full_func_name)
                    error(['Unit test: ' unit_test_path ' does not ' ...
                           'contain the function it is supposed to ' ...
                           'be testing: ' full_func_name])
                end

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

                % Remove unit test path
                unit_test_paths(strcmp(unit_test_path,unit_test_paths)) = [];
            else
                % Test is missing
                fprintf(2,['Doesnt exist: ' func_name '... \n']);
                missing = missing+1;
            end
        end    
        
        if ~isempty(unit_test_paths)
            error(['The following unknown unit tests were found: ' strjoin(unit_test_paths, newline)]);
        end
    end
end

% Print final message
disp('---');
msg = ['Test results: Passed - ' num2str(passed) '; Missing - ' num2str(missing) '; Failed - ' num2str(failed')];

% Return error if any tests are failed or missing; if you don't want to
% test a certain function for whatever reason, then just create an empty
% dummy test file with a comment containing the function is it supposed to
% be testing.
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
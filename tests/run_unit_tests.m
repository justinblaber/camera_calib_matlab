%% Clear
clc; clear;

%% Run tests

disp('---'); 
disp('Running unit tests...'); 

% Set directories in camera_calib to test; This script will only pass if
% each function in the following directories has a corresponding unit test
% which contains the function it is supposed to be testing.
camera_calib_sub_dir_names = {'+alg', ...
                              '+class', ...
                              '+util'};  
                          
% Check to make sure all camera_calib sub directory names start with a "+" 
% and exist.
tests_dir_path = fileparts(mfilename('fullpath'));
camera_calib_dir_path = fileparts(tests_dir_path);
for i = 1:length(camera_calib_sub_dir_names)
    if ~startsWith(camera_calib_sub_dir_names{i},'+')
        error(['camera_calib sub directory: "' ...
               camera_calib_sub_dir_names{i} '" must start with a "+"']);
    end
    
    camera_calib_sub_dir_path = fullfile(camera_calib_dir_path,camera_calib_sub_dir_names{i});
    if exist(camera_calib_sub_dir_path,'dir') ~= 7
        error(['camera_calib sub directory: "' ...
               camera_calib_sub_dir_path '" doesnt exist.']);
    end
end

% Make sure only directories exist in the unit tests folder
unit_tests_dir_path = fullfile(tests_dir_path,'unit');
l_unit_tests = dir_without_dots(unit_tests_dir_path);
if any(~[l_unit_tests.isdir])
    error('Only directories should exist in the unit test directory');
end

% Make sure camera_calib sub directory names and unit test sub directory
% names form a one to one correspondance
unit_tests_sub_dir_names = {l_unit_tests.name};          
if ~isequal(cellfun(@(x)x(2:end),camera_calib_sub_dir_names,'UniformOutput',false), ...
            unit_tests_sub_dir_names)
    error(['There is a mismatch between camera_calib sub directory names ' ...
           'and unit test sub directory names']);
end
                                  
% Keep track of passed, missing, and failed tests
passed = 0;
missing = 0;
failed = 0;

% Loop over camera_calib sub directories and run unit tests
for i = 1:length(camera_calib_sub_dir_names)
    disp('---'); 
    disp(['Directory:    ' camera_calib_sub_dir_names{i} '...']); 

    % Get corresponding unit test directory; remove "+" from directory name
    unit_tests_sub_dir_name = camera_calib_sub_dir_names{i}(2:end);
    unit_tests_sub_dir_path = fullfile(unit_tests_dir_path,unit_tests_sub_dir_name);
    
    % Get all unit tests in this directory
    l_unit_tests_sub = dir_without_dots(unit_tests_sub_dir_path);
    unit_test_paths = fullfile(unit_tests_sub_dir_path,{l_unit_tests_sub.name});

    % Get function to test
    camera_calib_sub_dir_path = fullfile(camera_calib_dir_path,camera_calib_sub_dir_names{i});
    l_camera_calib_sub = dir_without_dots(camera_calib_sub_dir_path);
    for j = 1:length(l_camera_calib_sub)
        % Only m-files are allowed in the camera_calib sub directories
        if l_camera_calib_sub(j).isdir || ~endsWith(l_camera_calib_sub(j).name,'.m')
            unknown_path = fullfile(camera_calib_sub_dir_path,l_camera_calib_sub(j).name);
            error(['Only m-files are allowed: "' unknown_path '"']);
        end
        
        % Get name without extension
        [~,func_name] = fileparts(l_camera_calib_sub(j).name);

        % Get corresponding unit test
        unit_test_path = fullfile(unit_tests_sub_dir_path,['test_' func_name '.m']);

        % Make sure unit test exists
        if exist(unit_test_path,'file') == 2
            % Make sure test actually contains the function
            full_func_name = strjoin({unit_tests_sub_dir_name,func_name},'.');
            if ~contains(fileread(unit_test_path),full_func_name)
                error(['Unit test: "' unit_test_path '" does not ' ...
                       'contain the function it is supposed to ' ...
                       'be testing: "' full_func_name '"'])
            end

            % Run the test
            try
                run(unit_test_path);   

                % Test passed if it gets here
                fprintf(['Passed:       ' func_name '...' newline]);
                passed = passed + 1;
            catch e
                % Test failed if it gets here
                fprintf(2,['Failed:       ' func_name '...' newline]);
                failed = failed + 1;
            end  

            % Remove unit test path
            unit_test_paths(strcmp(unit_test_path,unit_test_paths)) = [];
        else
            % Test is missing
            fprintf(2,['Doesnt exist: ' func_name '...' newline]);
            missing = missing+1;
        end
    end    

    if ~isempty(unit_test_paths)
        error(['The following unknown unit tests were found: ' ...
                strjoin(unit_test_paths, newline)]);
    end
end

% Print final message
disp('---');
msg = ['Test results: Passed - ' num2str(passed) '; Missing - ' num2str(missing) '; Failed - ' num2str(failed)];

% Return error if any tests are failed or missing; if you don't want to
% test a certain function for whatever reason, then just create an empty
% dummy test file with a comment containing the function it is supposed to
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

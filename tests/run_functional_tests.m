%% Clear
clc; clear;

%% Run tests

disp('---'); 
disp('Running functional tests...'); 

% Set tests to run
functional_test_names = {'single1', ...
                         'single2', ...
                         'single3', ...
                         'stereo1', ...
                         'stereo2', ...
                         'dot_vision'};  

tests_dir_path = fileparts(mfilename('fullpath'));
functional_tests_dir_path = fullfile(tests_dir_path,'functional');
for i = 1:length(functional_test_names)
    functional_test_path = fullfile(functional_tests_dir_path, ...
                                    functional_test_names{i});
    run(functional_test_path);
    pause
end
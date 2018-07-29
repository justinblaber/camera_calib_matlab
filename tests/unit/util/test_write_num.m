% Get temporary file
temp_path = tempname;

% Write a test num to this file
num = 1;
name = 'test';
util.write_num(num,name,temp_path);

% Check file contents
assert(strcmp(fileread(temp_path),[name ' = ' num2str(num) newline]));

% Remove temporary file
delete(temp_path);
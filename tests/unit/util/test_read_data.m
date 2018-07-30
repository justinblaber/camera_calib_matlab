% Get temporary file
temp_path = tempname;

% Create data file
name_s = 'string';
s1 = 'test1';
s2 = 'test2';

name_num = 'num';
num = 1;

name_array = 'array';
array = [1 2; 3 4];

test_str = ['%% test comment ' newline ...
            name_s ' = ' s1 newline ...
            name_s ' = ' s2 newline ...
            name_num ' = ' num2str(num) newline ...
            name_array ' = ' newline ...
            num2str(array(1,1)) ' ' num2str(array(1,2)) newline ...
            num2str(array(2,1)) ' ' num2str(array(2,2)) newline];    
       
% Write to temp file 
fid = fopen(temp_path,'w');
fprintf(fid,test_str);
fclose(fid);

% Read with read_data()
data = util.read_data(temp_path);

% Check string
assert(isfield(data,name_s));
assert(iscell(data.(name_s)))
assert(length(data.(name_s)) == 2);
assert(strcmp(data.(name_s){1},'test1'));
assert(strcmp(data.(name_s){2},'test2'));

% Check num
assert(isfield(data,name_num));
assert(data.(name_num) == num)

% Check array
assert(isfield(data,name_array));
assert(isequal(data.(name_array),array));

% Remove temporary file
delete(temp_path);
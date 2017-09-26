function array = read_array(file_path,name)
    % Reads array from file. File must have format:
    %   name=
    %   array
    %   ...
    % It is assumed the file being read from was written to with 
    % write_array().
    %  
    % Inputs:
    %   file_path - string; path to file to read array from.
    %   name - string; this is the name associated with the array.
    %
    % Outputs:
    %   array - array; contains array that matches name
    
    array = util.read_arrays(file_path,name);
    if length(array) ~= 1
        error(['Attempted to read single array with name: "' name '". ' ...
               'But ' num2str(length(array)) ' were found.']);
    end
    array = array{1};
end
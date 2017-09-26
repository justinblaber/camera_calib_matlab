function arrays = read_arrays(file_path,name)
    % Reads arrays from file. File must have format:
    %   name=
    %   array
    %   name=
    %   array
    %   ...
    % This function is plural because it reads all arrays which
    % match "name". It is assumed the file being read from was written to
    % with write_array(). 
    %  
    % Inputs:
    %   file_path - string; path to file to read arrays from.
    %   name - string; this is the name associated with the array.
    %
    % Outputs:
    %   arrays - cell; contains arrays that match name
            
    % Initialize
    arrays = {};
    
    % Go through line by line
    f = fopen(file_path);
    line = fgetl(f);
    line_num = 1;
    in_name = false;
    while ischar(line)
        line_split = strsplit(line,'=');
        if length(line_split) > 2 
            error(['Multiple assignments on line: ' num2str(line_num)]);
        elseif length(line_split) == 2 
            % This is a name
            if strcmp(line_split{1},name)
                % Line is the name we're interested in. Initialize new 
                % array and set in_name to true
                arrays{end+1} = []; %#ok<AGROW>
                in_name = true;
            else 
                % Line is a different name; we're no longer in the name 
                % we're interested in
                in_name = false;
            end
        else
            % These are not names
            if in_name
                % Attempt to concatenate array row
                try
                    arrays{end} = vertcat(arrays{end},str2num(line)); %#ok<ST2NM>
                catch e
                    error(['Failed to concatenate line: ' num2str(line_num) ' ' ...
                           'for matrix with name: "' name '". Are you sure ' ...
                           'this is a valid matrix? ' getReport(e)]);
                end
            end            
        end
        
        % Get next line
        line = fgetl(f);
        line_num = line_num+1;
    end
    fclose(f);
end
function [ r ] = unionRanges( ranges, margins )
%unionRanges Return the union of a set of ranges
%   Return the union of a set of ranges

    r = [];
    
    % If we're not merging anything, quit
    if isempty(ranges)
        return;
    end

    % Separate min and max values
    min_values = ranges(:, 1);
    max_values = ranges(:, 2);
    
    % Mark them
    s_ranges = size(ranges);
    min_values = cat(2, min_values, zeros(s_ranges(1, 1), 1));
    max_values = cat(2, max_values, ones(s_ranges(1, 1), 1));
    
    % Recombine them
    values = cat(1, max_values, min_values);
    
    % Sort the combined values descendingly
    values = sortrows(values, [-1 -2]);
    s_values = size(values);
    
    % Starting at the top, collapse them
    Stack = [];
    for i=1:s_values(1, 1)
        
        % If the stack is empty, add the next value; should always be max
        if isempty(Stack)
            Stack = cat(1, values(i, :), Stack);
            continue;
        end
        
        % If the stack is not empty, do a peek
        if Stack(1, 2)==1
            
            %
            % There is a max value on top of the stack
            %
            
            % If the incoming value is a min, pop
            if values(i, 2)==0
                
                tmp = Stack(1, :);
                Stack(1, :) = [];
                
                % If that pop resulted in an empty stack, save range
                if isempty(Stack)
                    r = cat(1, r, [values(i, 1) tmp(1, 1)]);
                end
                
            % Otherwise add the value to the stack
            else
                Stack = cat(1, values(i, :), Stack);
            end
            
        else
            
            %
            % There is a min value on top of the stack
            %
            
            % Just push the next value on
            Stack = cat(1, values(i, :), Stack);
            
        end
        
    end
    
    % Legacy support... dunno whether this is necessary
    r = sortrows(r);
    
end


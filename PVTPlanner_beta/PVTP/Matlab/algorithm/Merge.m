function [ V ] = Merge( S, margins )
%Merge Summary of this function goes here
%   Detailed explanation goes here

    % Merge information for each homotopic class
    modified = 1;
    while modified
        
        modified = 0;
        
        % Recalculate size
        s_S = size(S);
        
        for i=2:s_S(1, 2)
            
            for j=2:s_S(1, 2)

                % Don't try to merge with yourself
                if j==i
                    continue;
                end
                
                % Is element i a suffix of element j?
                if isSuffix(S(i).B, S(j).B)

                    % Get Interval information
                    min_i = min(S(i).Interval);
                    max_i = max(S(i).Interval);
                    min_j = min(S(j).Interval);
                    max_j = max(S(j).Interval);
                    
                    % Modify j
                    S(j).Interval = [min([min_i min_j]) max([max_i max_j])];
                    
                    % Mark as modified
                    modified = 1;
                    break;
                    
                end
                
            end
            
            % Element i was merged, break the loop and start over
            if modified
                S(i) = [];
                break;
            end
            
        end
        
    end
    
    % Gather final velocity intervals together
    s_S = size(S);
    V = [];
    for i=1:s_S(1, 2)
        
        if ~isempty(S(i).Interval)
            V = cat(1, V, S(i).Interval);
        end
        
    end
    
    % Perform a union of intervals
    V = unionRanges(V, margins);

end


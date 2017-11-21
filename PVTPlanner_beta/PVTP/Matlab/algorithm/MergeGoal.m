function [ S_g ] = MergeGoal( S, S_g, margins )
%MergeGoal Summary of this function goes here
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
                    
                    % Get interval from bounding trajectories
                    min_i = getStateCoord(S(i).UB(end, :), 't');
                    max_i = getStateCoord(S(i).LB(end, :), 't');
                    min_j = getStateCoord(S(j).UB(end, :), 't');
                    max_j = getStateCoord(S(j).LB(end, :), 't');
                    
                    % Merge lower-time bounding trajectory
                    if NumCompare(min_i, min_j, 'lt', margins)
                        S(j).UB = S(i).UB;
                    end
                    
                    % Merge upper-time bounding trajectory
                    if NumCompare(max_i, max_j, 'gt', margins)
                        S(j).LB = S(i).LB;
                    end
                    
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
    
    % If we're not merging anything, quit
    if isempty(S)
        return;
    end
    
    % Recalculate size of S
    s_S = size(S);
    for i=2:s_S(1, 2)
        S_g(end+1).UB = S(i).UB;
        S_g(end).LB = S(i).LB;
    end

end




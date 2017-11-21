function [ O ] = applyRandomObstacleActions( O )
%applyRandomObstacleActions Apply random actions to the obstacle set 
%   Apply random actions to the obstacle set

    time_distortion = 0.75;
    time_translation = 0.5;
    lane_width = 4;

    actions = [
        1;  % lane shift left
        2;  % lane shift right
        3;  % accelerate
        4   % decelerate
        ];
    s_actions = size(actions);
    s_O = size(O);
    random_nums = random('unid', s_actions(1,1), 1, s_O(1,1));
    for i=1:s_O(1,1)
        
        o = O(i,:);
        
        switch random_nums(i)
            
            % lane shift left
            case 1
                o = o + [-lane_width -lane_width 0 0];
                
            % lane shift right
            case 2
                o = o + [lane_width lane_width 0 0];
                
            % accelerate
            case 3
                max_t = getObstacleCoord(o, 'max_t');
                min_t = getObstacleCoord(o, 'min_t');
                t_size = time_distortion * (max_t - min_t);
                min_t = min_t - time_translation;
                max_t = min_t + t_size;
                o = [o(1:2) min_t max_t];
                
            % decelerate
            case 4
                max_t = getObstacleCoord(o, 'max_t');
                min_t = getObstacleCoord(o, 'min_t');
                t_size = (1+time_distortion) * (max_t - min_t);
                min_t = min_t + time_translation;
                max_t = min_t + t_size;
                o = [o(1:2) min_t max_t];
                
            otherwise
                
        end
        
        O(i,:) = o;
        
    end

end


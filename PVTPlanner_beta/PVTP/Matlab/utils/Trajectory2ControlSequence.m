function [ C ] = Trajectory2ControlSequence( T, margins )
%Trajectory2ControlSequence Convert a trajetory (a set of states) into a constrol sequence
%   Convert a trajetory (a set of states) into a constrol sequence

    C = [];
    
    s_T = size(T);
    if s_T(1,1) <= 1
        return;
    end

    a_min = getMargin(margins, 'acc_min');
    a_max = getMargin(margins, 'acc_max');
    
    for i=2:s_T(1,1)
        
        s1 = T(i-1, :);
        t1 = getStateCoord(s1, 't');
        v1 = getStateCoord(s1, 'v');
        
        s2 = T(i, :);
        t2 = getStateCoord(s2, 't');
        v2 = getStateCoord(s2, 'v');
        
        t_diff = t2 - t1;
        v_diff = v2 - v1;
        
        if NumCompare(t_diff, 0, 'eq', margins)
            continue;
        end
        
        if NumCompare(v_diff, 0, 'eq', margins)
            C = cat(1, C, [0 t_diff]);
            continue;
        end

        % calculate acceleration over this segment
        acc = (v2 - v1) / (t2 - t1);
        if NumCompare(acc, 0, 'eq', margins)
            acc = 0;
        elseif acc > 0
            acc = a_max;
        else
            acc = a_min;
        end
        
        % add to control set
        C = cat(1, C, [acc t_diff]);
        
    end

end


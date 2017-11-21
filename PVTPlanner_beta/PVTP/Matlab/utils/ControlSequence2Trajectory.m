function [ T ] = ControlSequence2Trajectory( C, v1, margins )
%ControlSequence2Trajectory Convert a sequence of controls into a sequence of states
%   Convert a sequence of controls into a sequence of states

    T = [];
    
    v_min = getMargin(margins, 'vel_min');
    v_max = getMargin(margins, 'vel_max');
    
    s_C = size(C);
    if isempty(C)
        return;
    end
    
    s1 = [0 0 v1];

    T = cat(1, T, s1);
    
    for i=1:s_C(1,1)
        
        x1 = getStateCoord(s1, 'x');
        t1 = getStateCoord(s1, 't');
        v1 = getStateCoord(s1, 'v');
        
        control = C(i, :);
        acc = control(1, 1);
        t_diff = control(1, 2);
        
        v2 = v1 + acc * t_diff;
        
        if NumCompare(v2, v_min, 'lt', margins)
            
            v2 = 0;
            t_diff_para = (-v1) / acc;
            x2 = motion(x1, v1, t_diff_para, acc);
            s2 = [
                x2 t1+t_diff_para v2;
                x2 t1+t_diff v2
                ];
            
        elseif NumCompare(v2, v_max, 'gt', margins)
            
            v2 = v_max;
            t_diff_para = (v2 - v1) / acc;
            x2_para = motion(x1, v1, t_diff_para, acc);
            x2 = x2_para + v2 * (t_diff - t_diff_para);
            s2 = [
                x2_para t1+t_diff_para v2;
                x2 t1+t_diff v2
                ];
        
        else
            
            x2 = motion(x1, v1, t_diff, acc);
            s2 = [x2 t1+t_diff v2];
        
        end
        
        T = cat(1, T, s2);
        
        s1 = s2(end,:);
        
    end

end


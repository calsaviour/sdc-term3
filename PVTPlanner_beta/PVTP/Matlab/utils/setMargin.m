function [ m ] = setMargin( val, margins, type )
%setMargin Set a unit in a margin
%   Set a unit in a margin

    m = margins;

    switch lower(type)
        case 'acc'
            m(1, 6:7) = val;
        case 'acc_min'
            m(1, 6) = val;
        case 'acc_max'
            m(1, 7) = val;
        case 'vel'
            m(1, 8:9) = val;
        case 'vel_min'
            m(1, 8) = val;
        case 'vel_max'
            m(1, 9) = val;
        case 'x_limit'
            m(1, 10) = val;      
        case 't_limit'
            m(1, 11) = val;
        case 'time_limit'
            m(1, 11) = val;
        case 'accuracy'
            m(1, 12) = val;
    end
    
end


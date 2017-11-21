function [ m ] = getMargin( margins, type )
%getMargin Get a unit from a margin
%   Get a unit from a margin

    switch lower(type)
        case 'ttc'
            m = margins(1, 1);
        case 'e_left'
            m = margins(1, 2);
        case 'e_right'
            m = margins(1, 3);
        case 'e_bottom'
            m = margins(1, 4);
        case 'e_top'
            m = margins(1, 5);
        case 'acc'
            m = margins(1, 6:7);
        case 'acc_min'
            range = getMargin(margins, 'acc');
            m = min(range);
        case 'acc_max'
            range = getMargin(margins, 'acc');
            m = max(range);
        case 'vel'
            m = margins(1, 8:9);
        case 'vel_min'
            range = getMargin(margins, 'vel');
            m = min(range);
        case 'vel_max'
            range = getMargin(margins, 'vel');
            m = max(range);
        case 'min_width'
            m = getMargin(margins, 'e_left') + getMargin(margins, 'e_right');
        case 'min_height'
            m = getMargin(margins, 'ttc') + getMargin(margins, 'e_bottom') + getMargin(margins, 'e_top');
        case 'x_limit'
            m = margins(1, 10);        
        case 't_limit'
            m = margins(1, 11);
        case 'time_limit'
            m = getMargin(margins, 't_limit');
        case 'accuracy'
            m = margins(1, 12);
    end
    
end


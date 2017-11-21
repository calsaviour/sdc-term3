function [ O_sorted ] = sortObstacles( O, column )
%sortObstacles Sort obstacles by a given column
%   Sort obstacles by a given column

    switch lower(column)
        case 'min_x'
            O_sorted = sortrows(O, 1);
        case 'max_x'
            O_sorted = sortrows(O, 2);
        case 'min_t'
            O_sorted = sortrows(O, 3);
        case 'max_t'
            O_sorted = sortrows(O, 4);
        case 'lower-right'
            O_sorted = sortrows(O, [3 -2]);
        case 'upper-left'
            O_sorted = sortrows(O, [1 -4]);
    end

end


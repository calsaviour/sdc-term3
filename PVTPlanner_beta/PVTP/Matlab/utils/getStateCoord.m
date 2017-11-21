function [ c ] = getStateCoord( s, coord )
%getStateCoord Get a coordinate from a state
%   Get a coordinate from a state

    switch lower(coord)
        case 'x'
            c = s(1, 1);
        case 't'
            c = s(1, 2);
        case 'v'
            c = s(1, 3);
    end

end


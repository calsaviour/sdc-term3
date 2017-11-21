function [ type ] = getEdgeType( R )
%getEdgeType Determine type of edge
%   Determine type of edge

    % Initial: 0
    % Internal: 1
    % Final: -2 lower, 2 upper
    % Free: 3

    s_R = size(R);
    type = R(1, s_R(1, 2));

end


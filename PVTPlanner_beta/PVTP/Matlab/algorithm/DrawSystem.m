function [ G, Goal ] = DrawSystem( p0, V_i, V_f, O, margins )
%DrawSystem Summary of this function goes here
%   Detailed explanation goes here

    [G Goal] = Forward( p0, V_i, V_f, O, margins );
    DrawSets(G, Goal, O, margins);

end


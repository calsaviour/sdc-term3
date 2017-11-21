function [ Y ] = quant( X, mu, sigma )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    Y = mu + sigma .* sqrt(2) .* erfinv( 2.*X - 1 );

end


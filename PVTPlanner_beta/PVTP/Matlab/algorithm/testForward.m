function [ G Goal ] = testForward( p0, V_i, V_f, O, margins )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    trials = 1;
    tic;
    for i=1:trials
        [G Goal] = Forward( p0, V_i, V_f, O, margins );
    end
    time = toc * 1000000;
    display([num2str(trials), ' trials, time elapsed: ', num2str(time), ' microseconds']);
    display(['Average per-iteration time: ', num2str(time/trials), ' microseconds']);
    
    s_G = size(G);
    cnt = 0;
    for i=1:s_G(1, 2)
        if ~isempty(G(i).V)
            %G(i).V
            %G(i).p
            cnt = cnt + 1;
        end
    end
    
    display(['Points with reachable intervals: ', num2str(cnt)]);

end


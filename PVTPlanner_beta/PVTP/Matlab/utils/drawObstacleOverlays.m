function [] = drawObstacleOverlays( total_num, traj, margins )
%drawObstacleOverlays Draw a series of obstacle overlays, write to disk
%   Draw a series of obstacle overlays, write to disk

    for i=0:total_num
        
        figure;
        hold on;
        xlabel('Path (m)');
        ylabel('Time (s)');
        
        drawObstacleOverlay( i, traj, margins );
        
        print('-dpng',['data/images/obs_', num2str(i)]);
        
        hold off;
        close;
        
        
    end

end


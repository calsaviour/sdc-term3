function screen2eps(filename)
%SCREEN2EPS Generate an EPS file of the current figure with
% dimensions consistent with the figure's screen dimensions.
%
% SCREEN2EPS('filename') saves the current figure to the
% EPS file "filename".
%
% Sean P. McCarthy
% Copyright (c) 1984-98 by MathWorks, Inc. All Rights Reserved

if nargin < 1
error('Not enough input arguments!')
end

oldscreenunits = get(gcf,'Units');
oldpaperunits = get(gcf,'PaperUnits');
oldpaperpos = get(gcf,'PaperPosition');
set(gcf,'Units','pixels');
scrpos = get(gcf,'Position');
newpos = scrpos/100;
set(gcf,'PaperUnits','inches',...
'PaperPosition',newpos)
print('-depsc2', filename);
drawnow
set(gcf,'Units',oldscreenunits,...
'PaperUnits',oldpaperunits,...
'PaperPosition',oldpaperpos)

end


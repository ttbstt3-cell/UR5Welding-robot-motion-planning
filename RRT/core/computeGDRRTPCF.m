function pcf = computeGDRRTPCF(successWindow)
% computeGDRRTPCF  Convert a ten-trial success window to collision frequency.
%
% successWindow is true for a collision-free base expansion.  Therefore pcf
% is deliberately (10-p)/10, not p/10.

successWindow = logical(successWindow(:));
if numel(successWindow) ~= 10
    error('successWindow must contain exactly ten expansion outcomes.');
end

p = sum(successWindow);
pcf = (10-p)/10;

end

function errorMm=contour_distance_copy(P,A,decisionThreshold)
% Exact all-segment point/polyline distances, bounded-memory vectorization.
% Every point is compared with EVERY original segment, not a local window.
if nargin<3, decisionThreshold=.5; end
origin=P(1:end-1,:); v=diff(P); vv=max(sum(v.^2,2),eps).';
errorMm=zeros(size(A,1),1);
for first=1:256:size(A,1)
    idx=first:min(first+255,size(A,1)); X=A(idx,:);
    dx=X(:,1)-origin(:,1).'; dy=X(:,2)-origin(:,2).'; dz=X(:,3)-origin(:,3).';
    u=max(0,min(1,(dx.*v(:,1).'+dy.*v(:,2).'+dz.*v(:,3).')./vv));
    ds=(dx-u.*v(:,1).').^2+(dy-u.*v(:,2).').^2+(dz-u.*v(:,3).').^2;
    errorMm(idx)=1000*sqrt(min(ds,[],2));
end
% Preserve the original scalar >= threshold decision at roundoff boundaries.
% This is an exact recomputation dispatch, NOT an acceptance tolerance.
for i=find(abs(errorMm-decisionThreshold)<1e-8).'
    best=Inf;
    for k=1:size(P,1)-1
        d=P(k+1,:)-P(k,:); u=max(0,min(1,dot(A(i,:)-P(k,:),d)/max(dot(d,d),eps)));
        best=min(best,norm(A(i,:)-(P(k,:)+u*d)));
    end
    errorMm(i)=1000*best;
end
end

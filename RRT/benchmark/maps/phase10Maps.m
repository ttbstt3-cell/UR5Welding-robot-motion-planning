function maps = phase10Maps()
% phase10Maps  Fixed 2-D maps for Phase 10A/10B experiments.
%
% Every map uses the common project-normalized workspace and start/goal.
% Obstacles are fixed here; benchmark runs never generate them randomly.

maps = repmat(struct( ...
    'id',0, ...
    'name','', ...
    'mapBounds',[0 100 0 100], ...
    'start',[10 10], ...
    'goal',[90 90], ...
    'obstacles',zeros(0,4), ...
    'clearance',0),4,1);

maps(1).id = 1;
maps(1).name = 'Simple';
maps(1).obstacles = [
    45 30 10 10
    70 60 10 10];

maps(2).id = 2;
maps(2).name = 'StartGoalNeighborhood';
maps(2).obstacles = [
    12  0 10  8
     0 12  8 10
    82 72  8  8
    72 82  8  8];

maps(3).id = 3;
maps(3).name = 'DenseClutter';
maps(3).obstacles = [
    25 10 10 25
    45  0 10 25
    65 10 10 25
    25 65 10 25
    45 75 10 25
    65 65 10 25
    35 40 10 10
    55 50 10 10];

maps(4).id = 4;
maps(4).name = 'NarrowPassageU';
maps(4).obstacles = [
    30 20 40  8
    30 20  8 48
    62 20  8 48];

end

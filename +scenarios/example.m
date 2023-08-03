%% Example of generating multiple UEs and targets
% Note that the increased number of UEs and targets will significantly
% lower the speed of simulation.

% Multiple UEs
ue(1) = networkElements.ue.basicUE();
ue(1).ID       = 1;
ue(1).position = [100 100 1.5];  % in meters

ue(2) = networkElements.ue.basicUE();
ue(2).ID       = 1;
ue(2).position = [200 -50 1.5];  % in meters

ueParams = ue;

% Multiple targets
tgt(1) = networkElements.target.basicTarget();
tgt(1).ID       = 1;
tgt(1).position = [100 100 1.5];  % in meters
tgt(1).rcs      = 1;              % in meters^2
tgt(1).velocity = 5;              % in meters per second

tgt(2) = networkElements.target.basicTarget();
tgt(2).ID       = 2;
tgt(2).position = [200 -50 1.5];  % in meters
tgt(2).rcs      = 1;              % in meters^2
tgt(2).velocity = -5;             % in meters per second

tgtParams = tgt;

%
% TestScript for Assignment 1
%

%% Define a small map
map = false(10);

% Add an obstacle
map (1:5,5:8) = true;

start_coords = [2, 4];
dest_coords  = [1, 9];

%%
close all;
%[route, numExpanded] = DijkstraGrid (map, start_coords, dest_coords);
% Uncomment following line to run Astar
[route, numExpanded] = AStarGrid (map, start_coords, dest_coords);



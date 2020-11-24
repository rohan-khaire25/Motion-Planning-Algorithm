function [route,numExpanded] = DijkstraGrid (input_map, start_coords, dest_coords)
% Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.
% Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    numExpanded: Remember to also return the total number of nodes
%    expanded during your search. Do not count the goal node as an expanded node.


% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination
function [n, s, e, w] = neighbors(current_node)
        [i, j] = ind2sub(size(distanceFromStart), current);
        if i == 1 && j == 1
            s = current_node + 1;
            w = current_node + nrows;
            n = current_node + (nrows - 1);
            e = current_node + (ncols-1) * nrows;
        elseif i == nrows && j == 1
            n = current_node - 1;
            w = current_node + nrows;
            s = current_node - (nrows - 1);
            e = current_node + (ncols-1) * nrows;
        elseif i == nrows && j == ncols
            e = current_node - nrows;
            n = current_node - 1;
            s = current_node - (nrows - 1);
            w = current_node - (ncols-1) * nrows;
        elseif i == 1 && j == ncols
            e = current_node - nrows;
            s = current_node + 1;
            n = current_node + (nrows - 1);
            w = current_node - (ncols-1) * nrows;
        elseif i == 1 && j > 1
            s = current_node + 1;
            e = current_node - nrows;
            w = current_node + nrows;
            n = current_node + (nrows - 1);
        elseif i == nrows && j > 1
            n = current_node - 1;
            e = current_node - nrows;
            w = current_node + nrows;
            s = current_node - (nrows - 1);
        elseif i > 1 && j == 1
            n = current_node - 1;
            s = current_node + 1;
            w = current_node + nrows;
            e = current_node + (ncols-1) * nrows;
        elseif i > 1 && j == ncols
            n = current_node - 1;
            s = current_node + 1;
            e = current_node - nrows;
            w = current_node - (ncols-1) * nrows;
        else
            n = current_node - 1;
            s = current_node + 1;
            e = current_node - nrows;
            w = current_node + nrows;
            
        end
        
end

    function manhattan_distance(node)
    [x, y] = ind2sub(size(distanceFromStart), unexplored(index));
    dist = abs(start_coords(1)-x) + abs(start_coords(2)-y);
    if dist < 0
        distanceFromStart(node) = NaN;
    else
        distanceFromStart(node) = dist
    end
end

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0; ...
	0.5 0.5 0.5];

colormap(cmap);

% variable to control if the map is being visualized on every
% iteration
drawMapEveryTime = true;

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;   % Mark free cells
map(input_map)  = 2;   % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize distance array
distanceFromStart = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distanceFromStart(start_node) = 0;

% keep track of number of nodes expanded 
numExpanded = 0;

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    % make drawMapEveryTime = true if you want to see how the 
    % nodes are expanded on the grid. 
    if (drawMapEveryTime)
        image(1.5, 1.5, map);
        grid on;
        axis image;
        drawnow;
    end
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distanceFromStart(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end
    
    % Update map
    map(current) = 3;         % mark current node as visited
    distanceFromStart(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    
    [i, j] = ind2sub(size(distanceFromStart), current);
   
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.

    [n,s,e,w] = neighbors(current);
    
    unexplored = [n,s,w,e];
    for index = 1:length(unexplored)
        count = 0;
        if map(unexplored(index)) == 1 || map(unexplored(index)) == 6 
            manhattan_distance(unexplored(index))
            map(unexplored(index)) = 4;  
        end
        if unexplored(index) ~= start_node && parent(unexplored(index)) == 0 && map(unexplored(index)) ~= 2
            parent(unexplored(index)) = current
        end
        route1= [];
        route1 = [unexplored(index)];
        while (parent(route1(1)) ~= 0)
            route1 = [parent(route1(1)), route1];
            count = count + 1;
        end
        if map(unexplored(index)) == 4
            distanceFromStart(unexplored(index)) = count
        end
 
    
    end
    
        
        
    
 

end

%% Construct route from start to dest by following the parent links
if (isinf(distanceFromStart(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
    
        % Snippet of code used to visualize the map and the path
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end

end

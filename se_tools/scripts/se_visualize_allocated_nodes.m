% Copyright 2019 Sotiris Papatheodorou
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%    http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

% Visualize the text file written by calling Octree::writeAllocatedNodes().
% This will show all allocated Nodes (and VoxelBlocks) as colored cubes in 3D.
% Visualizing the cubes may take a while, especially in large volumes.

clear variables
close all

addpath(genpath('octave_functions'));



% Settings.
filename = '~/nodes.txt';
plot_coordinates        = false;
plot_coordinate_markers = false;



% List of colors.
colors = [1.0 0.0 0.0 ;
          0.0 1.0 0.0 ;
          0.0 0.0 1.0];



% Load the allocated Node list.
data = importdata(filename);
num_nodes = size(data, 1);
max_side = max(data(:, 4));
fprintf('Loaded %d nodes\n', num_nodes);
fflush(stdout);



figure;
hold on;
axis equal;
grid on;

% Iterate over all Nodes.
for i = 1:num_nodes
	% Get the data for the current Node.
	node_x    = data(i, 1);
	node_y    = data(i, 2);
	node_z    = data(i, 3);
	node_side = data(i, 4);

	% Compute the Node level.
	level = log2(max_side / node_side);

	% Create a unit cube centered at the origin.
	[xc, yc, zc] = cube();
	% Translate so the bottom corner is at the origin.
	xc = xc + 0.5;
	yc = yc + 0.5;
	zc = zc + 0.5;
	% Scale with the node side length.
	xc = node_side * xc;
	yc = node_side * yc;
	zc = node_side * zc;
	% Translate to the Node's position.
	xc = xc + node_x;
	yc = yc + node_y;
	zc = zc + node_z;

	% Plot the cube.
	color_index = mod(level, size(colors, 1)) + 1;
	mesh(xc, yc, zc, 'FaceAlpha', 0, 'EdgeColor', colors(color_index, :));

	% Show the Node coordinates.
	if plot_coordinate_markers
		plot3(node_x, node_y, node_z, 'kx');
	end
	if plot_coordinates
		text(node_x, node_y, node_z, ...
				sprintf('(%d,%d,%d)', node_x, node_y, node_z));
	end
end


% CUBE  Generate cube.
%    CUBE draws the unit cube centered on the origin.
%
%    [X, Y, Z] = CUBE returns the unit cube vertices which can be plotted using
%    mesh(X, Y, Z).


% Copyright (C) 2019 Sotiris Papatheodorou
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

function [xx, yy, zz] = cube()
	% Compute the vertices of the unit cube centered on [0.5 0.5 0.5]'.
	x = [];
	y = [];
	z = [];
	% x- face.
	x = [x; 0 0 0 0 0];
	y = [y; 0 1 1 0 0];
	z = [z; 0 0 1 1 0];
	% x+ face.
	x = [x; 1 1 1 1 1];
	y = [y; 0 1 1 0 0];
	z = [z; 0 0 1 1 0];
	% y- face.
	x = [x; 0 1 1 0 0];
	y = [y; 0 0 0 0 0];
	z = [z; 0 0 1 1 0];
	% y+ face.
	x = [x; 0 1 1 0 0];
	y = [y; 1 1 1 1 1];
	z = [z; 0 0 1 1 0];
	% z- face
	x = [x;0 1 1 0 0];
	y = [y;0 0 1 1 0];
	z = [z;0 0 0 0 0];
	% z+ face
	x = [x; 0 1 1 0 0];
	y = [y; 0 0 1 1 0];
	z = [z; 1 1 1 1 1];

	% Center cube on [0 0 0]'.
	x = x - 0.5;
	y = y - 0.5;
	z = z - 0.5;

	if nargout == 0
		% Draw the cube.
		mesh(x, y, z);
	else
		% Return the cube vertices.
		xx = x;
		yy = y;
		zz = z;
	end
end


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

function plot_axes(T, line_width)
	if nargin == 0
		T = eye(4);
		line_width = 0.5;
	elseif nargin == 1
		line_width = 0.5;
	end

	%if abs(det(T(1:3, 1:3)) - 1) > 100 * eps
	%	fprintf('Error: Invalid axes, determinant is not 1\n');
	%end

	% Save the hold state and enable hold.
	current_hold_state = ishold;
	hold on;

	origin = T(1:3,4);
	x_axis = T(1:3,1);
	y_axis = T(1:3,2);
	z_axis = T(1:3,3);

	plot3(  [origin(1) origin(1) + x_axis(1)], ...
			[origin(2) origin(2) + x_axis(2)], ...
			[origin(3) origin(3) + x_axis(3)], 'r', 'LineWidth', line_width);
	plot3(  [origin(1) origin(1) + y_axis(1)], ...
			[origin(2) origin(2) + y_axis(2)], ...
			[origin(3) origin(3) + y_axis(3)], 'g', 'LineWidth', line_width);
	plot3(  [origin(1) origin(1) + z_axis(1)], ...
			[origin(2) origin(2) + z_axis(2)], ...
			[origin(3) origin(3) + z_axis(3)], 'b', 'LineWidth', line_width);

	% Restore the hold state if necessary.
	if ~current_hold_state
		hold off;
	end
end


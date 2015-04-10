function [y, dy] = linear_interp(x0, y0, x, dx)
%LINEAR_INTERP Linear interpolation between values.
%
% Copyright 2015 Mikhail S. Jones

  % Limit range since curve fit is only valid within range
  x = clamp(x, x0(1), x0(end));

  % Initialize ouputs
  y = y0(1); dy = 0;

  % Loop through intervals
  for i = 1:numel(x0)-1
    % Find interval covering current point
    if x >= x0(i) && x <= x0(i+1)
      % Compute coefficients
      a0 = y0(i);
      a1 = (y0(i+1) - y0(i))/(x0(i+1) - x0(i));
      s = (x - x0(i));

      % Compute interpolated values
      y = a0 + a1*s;
      dy = dx*a1;
    end % if
  end % for
end % linear_interp

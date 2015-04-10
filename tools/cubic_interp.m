function [y, dy] = cubi_interp(x0, y0, dy0, x, dx)
%CUBIC_INTERP Cubic interpolation between values.
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
      % Interpolate
      a3 = 2*(y0(i) - y0(i+1)) + (dy0(i) + dy0(i+1))*(x0(i+1) - x0(i));
      a2 = y0(i+1) - y0(i) - dy0(i)*(x0(i+1) - x0(i)) - a3;
      a1 = dy0(i)*(x0(i+1) - x0(i));
      a0 = y0(i);
      s = (x - x0(i))/(x0(i+1) - x0(i));
      y = a0 + a1*s + a2*s^2 + a3*s^3;
      dy = dx*(- a1/(x0(i) - x0(i+1)) + 2*a2*(x - x0(i))/(x0(i) - x0(i+1))^2 - 3*a3*(x - x0(i))^2/(x0(i) - x0(i+1))^3);
    end % if
  end % for
end % cubic_interp

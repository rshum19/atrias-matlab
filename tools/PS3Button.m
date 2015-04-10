%PS3BUTTON PS3 controller button object.
%
% Copyright 2015 Mikhail S. Jones

classdef PS3Button < handle

  % PROTECTED PROPERTIES ==================================================
  properties (SetAccess = protected)
    value@logical = false % Current button state
    isPressed@logical = false % Button was just pushed
    isReleased@logical = false % Button was just released
    clickDuration@double = 0 % Duration button has been held down
    clickCount@double = 0 % Number of button presses

    time@double = 0 % Time since initialization (s)
    timePressed@double = 0 % Time at which button was last pressed (s)
    
    sampleRate@double = 0.001 % Fixed sample time (s)
    clickInterval@double = 0.5 % Acceptable interval to count click (s)
  end % properties

  % PUBLIC METHODS ========================================================
  methods
    function update(obj, value)
    %UPDATE Update PS3 button object with controller data.

      % Update time
      obj.time = obj.time + obj.sampleRate;

      % Check if button was pressed
      obj.isPressed = ~obj.value && value;
      
      % Check if button was released
      obj.isReleased = obj.value && ~value;

      % Update click duration
      if obj.isPressed
        obj.timePressed = obj.time;
      else
        obj.clickDuration = value*(obj.time - obj.timePressed);
      end % if

      % Update click count
      if (obj.time - obj.timePressed) < obj.clickInterval
        obj.clickCount = obj.clickCount + obj.isPressed;
      else
        obj.clickCount = 0;
      end % if

      % Update button state
      obj.value = value;
    end % update
  end % methods
end % classdef

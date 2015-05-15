%PS3CONTROLLER PS3 controller object.
%
% Copyright 2015 Mikhail S. Jones

classdef PS3Controller < handle

  % PROTECTED PROPERTIES ==================================================
  properties (SetAccess = protected)
    % Joysticks
    leftStickX@double = 0
    leftStickY@double = 0
    rightStickX@double = 0
    rightStickY@double = 0

    % Buttons
    select@PS3Button
    l3@PS3Button
    r3@PS3Button
    start@PS3Button
    up@PS3Button
    right@PS3Button
    down@PS3Button
    left@PS3Button
    l2@PS3Button
    r2@PS3Button
    l1@PS3Button
    r1@PS3Button
    triangle@PS3Button
    circle@PS3Button
    cross@PS3Button
    square@PS3Button
    ps@PS3Button
  end % properties

  % PUBLIC METHODS ========================================================
  methods
    function obj = PS3Controller
    %PS3CONTROLLER PS3 controller class constructor.

      % Initialize buttons
      obj.select = PS3Button;
      obj.l3 = PS3Button;
      obj.r3 = PS3Button;
      obj.start = PS3Button;
      obj.up = PS3Button;
      obj.right = PS3Button;
      obj.down = PS3Button;
      obj.left = PS3Button;
      obj.l2 = PS3Button;
      obj.r2 = PS3Button;
      obj.l1 = PS3Button;
      obj.r1 = PS3Button;
      obj.triangle = PS3Button;
      obj.circle = PS3Button;
      obj.cross = PS3Button;
      obj.square = PS3Button;
      obj.ps = PS3Button;
    end % PS3Controller

    function update(obj, axes, buttons)
    %UPDATE Update PS3 controller object with controller data.

      % Update joysticks
      obj.leftStickX = -axes(1);
      obj.leftStickY = -axes(2);
      obj.rightStickX = -axes(3);
      obj.rightStickY = -axes(4);

      % Convert button vector to logical
      buttons = logical(buttons);

      % Update button objects
      obj.select.update(buttons(1));
      obj.l3.update(buttons(2));
      obj.r3.update(buttons(3));
      obj.start.update(buttons(4));
      obj.up.update(buttons(5));
      obj.right.update(buttons(6));
      obj.down.update(buttons(7));
      obj.left.update(buttons(8));
      obj.l2.update(buttons(9));
      obj.r2.update(buttons(10));
      obj.l1.update(buttons(11));
      obj.r1.update(buttons(12));
      obj.triangle.update(buttons(13));
      obj.circle.update(buttons(14));
      obj.cross.update(buttons(15));
      obj.square.update(buttons(16));
      obj.ps.update(buttons(17));
    end % update
  end % methods
end % classdef

function output = ControlLaw(input,~)
    % The input is the simple vehicle state variables
    x = [input(2);input(3);input(5);input(6)];
    % Reference

    % Vehicle longitudinal position
    X = input(1);

    %
    LateralDisp = 3.6;

    if X <= 15
        r = 0;
    end
    if X > 15 && X <= 70
        r = LateralDisp;
    end
    if X > 70
        r = 0;
    end

    % Control gain
    K = [0.7936    6.6882    1.6107    0.5090];
    u = - K*x + K(1)*r;

    % Saturation at 70 deg
    if abs(u) < 42*pi/180
        output = u;
    else
        output = sign(u)*42*pi/180;
    end

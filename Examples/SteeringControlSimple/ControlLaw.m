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

    % Control gain LQR
    K = [0.5477    4.4651    1.0744    0.8169];
    u = - K*x + K(1)*r;

    % Saturation
    if abs(u) < 42*pi/180
        output = u;
    else
        output = sign(u)*42*pi/180;
    end

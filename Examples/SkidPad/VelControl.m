function output = VelControl(input,~)
    % Current velocity
    vel = input(4);
    % Reference velocity
    velRef = 8.333;

    % Control gain
    K = 100000;

    output = K * (velRef - vel);

function bezierOutput = bezierCurveSmoothing(pathObj)

    %smoothing parameters
    ratio1 = 0.9;
    ratio2 = 0.1;
    stepSize = 0.001;

    numStates = pathObj.NumStates;
    xStates = pathObj.States(:, 1);
    yStates = pathObj.States(:, 2);
    ss = pathObj.StateSpace;

    Px = [];
    Py = [];
    Bx = xStates(1);
    By = yStates(1);

    for k=2:numStates-1
        state1 = [xStates(k-1), yStates(k-1), 0];
        state2 = [xStates(k), yStates(k), 0];
        state3 = [xStates(k+1), yStates(k+1), 0];

        controlPoint1 = ss.interpolate(state1, state2, ratio1);
        controlPoint2 = ss.interpolate(state2, state3, ratio2);

        states = [controlPoint1; state2; controlPoint2];

        controlPoints = 3;
        a = zeros(1, controlPoints);
        px = states(:, 1);
        py = states(:, 2);
    
        kx = zeros(1, controlPoints);
        ky = zeros(1, controlPoints);
    

        bx = zeros(1, 1/stepSize);
        bx(1) = px(1);
        bx(end) = px(end);
    
        by = zeros(1, 1/stepSize);
        by(1) = py(1);
        by(end) = py(end);
    
        t = 0:stepSize:1;
    
        for i=1:controlPoints
            a(i) = factorial(controlPoints - 1)/(factorial(i - 1)*factorial(controlPoints - i));
        end
    
        for j=2:(1/stepSize)
            x = 0;
            y = 0;
    
            for s=1:controlPoints
                kx(s) = a(s)*t(j).^(s - 1)*(1 - t(j)).^(controlPoints - s)*px(s);
                ky(s) = a(s)*t(j).^(s - 1)*(1 - t(j)).^(controlPoints - s)*py(s);
    
                x = x + kx(s);
                y = y + ky(s);
            end
    
            bx(j) = x;
            by(j) = y;
        end
    
        Px = [Px; px];
        Py = [Py; py];
        Bx = [Bx, bx];
        By = [By, by];
    end

    Bx = [Bx, xStates(end)];
    By = [By, yStates(end)];

    bezierOutput = struct;
    bezierOutput.px = Px;
    bezierOutput.py = Py;
    bezierOutput.bx = Bx;
    bezierOutput.by = By;
end
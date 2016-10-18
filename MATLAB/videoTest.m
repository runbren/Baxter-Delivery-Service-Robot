while 1
    figure(1)
    steeringDir = step(VFH, ranges, angles, theta);
    show(VFH)
    
end
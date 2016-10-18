vfh = robotics.VectorFieldHistogram;

ranges = 10*ones(1,500);
ranges(1,225:275) = 1.5;
angles = linspace(-pi,pi,500);
targetDir = 0;

steeringDir = step(vfh,ranges, angles, targetDir)

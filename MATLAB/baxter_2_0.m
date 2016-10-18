odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 4];
rangeFinderModel.Map = map;

tftree = rostf;
waitForTransform(tftree,'/base_link','/camera_depth_frame');

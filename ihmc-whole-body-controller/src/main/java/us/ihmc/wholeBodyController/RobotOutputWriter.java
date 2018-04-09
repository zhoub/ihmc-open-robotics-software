package us.ihmc.wholeBodyController;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface RobotOutputWriter
{
   void initialize();

   void processAfterController(long timestamp);

   void setLowLevelControllerCoreOutput(FullRobotModel controllerRobotModel, JointDesiredOutputList lowLevelControllerCoreOutput,
                                        RawJointSensorDataHolderMap rawJointSensorDataHolderMap);

   void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController);

   YoVariableRegistry getControllerYoVariableRegistry();
}

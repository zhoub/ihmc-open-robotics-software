package us.ihmc.sensorProcessing.outputProcessing;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface RobotOutputProcessor
{
   void initialize();
   
   void processAfterController();

   void setLowLevelControllerCoreOutput(FullRobotModel controllerRobotModel, JointDesiredOutputList lowLevelControllerCoreOutput);

   YoVariableRegistry getControllerYoVariableRegistry();
}

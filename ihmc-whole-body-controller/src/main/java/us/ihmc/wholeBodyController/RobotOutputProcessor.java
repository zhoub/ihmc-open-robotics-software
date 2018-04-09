package us.ihmc.wholeBodyController;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface RobotOutputProcessor
{
   void initialize();

   /**
    * Processes the outputs from the controller before sending them to the output writer
    * @param timestamp current robot timestamp in nanoseconds
    */
   void processAfterController(long timestamp);

   void setLowLevelControllerOutput(FullRobotModel controllerRobotModel, JointDesiredOutputList lowLevelControllerCoreOutput);

   YoVariableRegistry getYoVariableRegistry();
}

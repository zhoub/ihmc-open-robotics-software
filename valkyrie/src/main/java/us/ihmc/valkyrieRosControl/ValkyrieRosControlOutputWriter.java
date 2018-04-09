package us.ihmc.valkyrieRosControl;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.wholeBodyController.RobotOutputWriter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieRosControlOutputWriter implements RobotOutputWriter, ControllerStateChangedListener, ControllerFailureListener
{

   public ValkyrieRosControlOutputWriter(ValkyrieRobotModel robotModel)
   {
      // TODO Auto-generated constructor stub
   }

   @Override
   public void initialize()
   {
      
   }

   @Override
   public void processAfterController(long timestamp)
   {
      
   }

   @Override
   public void setLowLevelControllerCoreOutput(FullRobotModel controllerRobotModel, JointDesiredOutputList lowLevelControllerCoreOutput, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController)
   {
      
   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return null;
   }

   @Override
   public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState)
   {
      
   }

   @Override
   public void controllerFailed(FrameVector2D fallingDirection)
   {
      
   }

}

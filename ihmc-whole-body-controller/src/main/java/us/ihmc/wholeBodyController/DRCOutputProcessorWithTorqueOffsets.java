package us.ihmc.wholeBodyController;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;

public class DRCOutputProcessorWithTorqueOffsets implements DRCOutputProcessor, JointTorqueOffsetProcessor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DRCOutputProcessor outputProcessor;

   private final YoDouble alphaTorqueOffset = new YoDouble("alphaTorqueOffset", "Filter for integrating acceleration to get a torque offset at each joint",
                                                           registry);

   private final YoBoolean resetTorqueOffsets = new YoBoolean("resetTorqueOffsets", registry);

   private PairList<JointDesiredOutput, YoDouble> torqueOffsetList;
   private HashMap<OneDoFJoint, YoDouble> torqueOffsetMap;

   private final double updateDT;

   public DRCOutputProcessorWithTorqueOffsets(DRCOutputProcessor outputProcessor, double updateDT)
   {
      this.updateDT = updateDT;
      this.outputProcessor = outputProcessor;
      if (outputProcessor != null)
      {
         registry.addChild(outputProcessor.getControllerYoVariableRegistry());
      }
   }

   @Override
   public void initialize()
   {
      if (outputProcessor != null)
      {
         outputProcessor.initialize();
      }
   }

   @Override
   public void processAfterController(long timestamp)
   {
      for (int i = 0; i < torqueOffsetList.size(); i++)
      {
         JointDesiredOutput jointData = torqueOffsetList.first(i);
         YoDouble torqueOffsetVariable = torqueOffsetList.second(i);

         double desiredAcceleration = jointData.hasDesiredAcceleration() ? jointData.getDesiredAcceleration() : 0.0;

         if (resetTorqueOffsets.getBooleanValue())
            torqueOffsetVariable.set(0.0);

         double offsetTorque = torqueOffsetVariable.getDoubleValue();
         double ditherTorque = 0.0;

         double alpha = alphaTorqueOffset.getDoubleValue();
         offsetTorque = alpha * (offsetTorque + desiredAcceleration * updateDT) + (1.0 - alpha) * offsetTorque;
         torqueOffsetVariable.set(offsetTorque);
         double desiredTorque = jointData.hasDesiredTorque() ? jointData.getDesiredTorque() : 0.0;
         jointData.setDesiredTorque(desiredTorque + offsetTorque + ditherTorque);
      }

      if (outputProcessor != null)
      {
         outputProcessor.processAfterController(timestamp);
      }
   }

   @Override
   public void setLowLevelControllerCoreOutput(FullRobotModel controllerRobotModel, JointDesiredOutputList lowLevelControllerCoreOutput)
   {
      if (outputProcessor != null)
      {
         outputProcessor.setLowLevelControllerCoreOutput(controllerRobotModel, lowLevelControllerCoreOutput);
      }

      torqueOffsetList = new PairList<>();
      torqueOffsetMap = new HashMap<>();

      for (int i = 0; i < lowLevelControllerCoreOutput.getNumberOfJointsWithDesiredOutput(); i++)
      {
         JointDesiredOutput jointData = lowLevelControllerCoreOutput.getJointDesiredOutput(i);
         final YoDouble torqueOffset = new YoDouble("tauOffset_" + lowLevelControllerCoreOutput.getJointName(i), registry);

         torqueOffsetList.add(jointData, torqueOffset);
         torqueOffsetMap.put(lowLevelControllerCoreOutput.getOneDoFJoint(i), torqueOffset);
      }

   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void subtractTorqueOffset(OneDoFJoint oneDoFJoint, double torqueOffset)
   {
      YoDouble torqueOffsetVariable = torqueOffsetMap.get(oneDoFJoint);
      torqueOffsetVariable.sub(torqueOffset);
   }
}

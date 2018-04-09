package us.ihmc.sensorProcessing.outputProcessors;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

import java.util.ArrayList;

public class OutputProcessorFactory
{
   private final RequiredFactoryField<FullRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<JointDesiredOutputList> controllerOutput = new RequiredFactoryField<>("controllerOutput");
   private final ArrayList<RobotOutputProcessor> outputProcessors;

   public OutputProcessorFactory()
   {
      this.outputProcessors = new ArrayList<>();
   }

   public void setLowLevelControllerOutput(FullRobotModel fullRobotModel, JointDesiredOutputList controllerOutput)
   {
      outputProcessors.clear();
      this.fullRobotModel.set(fullRobotModel);
      this.controllerOutput.set(controllerOutput);
   }

   public void addComponent(RobotOutputProcessor outputProcessor)
   {
      outputProcessor.setLowLevelControllerOutput(fullRobotModel.get(), controllerOutput.get());
      outputProcessors.add(outputProcessor);
   }

   public RobotOutputProcessor build()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);
      FactoryTools.disposeFactory(this);

      return new ModularOutputProcessor("modularOutputProcessor", outputProcessors);
   }
}

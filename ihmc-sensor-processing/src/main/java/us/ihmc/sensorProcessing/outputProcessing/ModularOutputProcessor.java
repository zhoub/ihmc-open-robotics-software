package us.ihmc.sensorProcessing.outputProcessing;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;

public class ModularOutputProcessor implements RobotOutputProcessor
{
   private final ArrayList<RobotOutputProcessor> outputProcessors = new ArrayList<>();
   private final YoVariableRegistry registry;

   public ModularOutputProcessor(String name, RobotOutputProcessor outputProcessor)
   {
      this(name);
      addOutputProcessor(outputProcessor);
   }

   public ModularOutputProcessor(String name, RobotOutputProcessor[] outputProcessors)
   {
      this(name);

      for (RobotOutputProcessor outputProcessor : outputProcessors)
      {
         addOutputProcessor(outputProcessor);
      }
   }

   public ModularOutputProcessor(String name, ArrayList<RobotOutputProcessor> outputProcessors)
   {
      this(name);

      for (RobotOutputProcessor outputProcessor : outputProcessors)
      {
         addOutputProcessor(outputProcessor);
      }
   }

   public ModularOutputProcessor(String name)
   {
      this.registry = new YoVariableRegistry(name);
   }

   public void addOutputProcessor(RobotOutputProcessor outputProcessor)
   {
      this.outputProcessors.add(outputProcessor);
      this.registry.addChild(outputProcessor.getYoVariableRegistry());
   }

   @Override
   public void initialize()
   {
      for (int i = 0; i < outputProcessors.size(); i++)
      {
         outputProcessors.get(i).initialize();
      }
   }

   @Override
   public void processAfterController()
   {
      for (int i = 0; i < outputProcessors.size(); i++)
      {
         outputProcessors.get(i).processAfterController();
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void setLowLevelControllerOutput(FullRobotModel controllerRobotModel, JointDesiredOutputList lowLevelControllerCoreOutput)
   {
      for (int i = 0; i < outputProcessors.size(); i++)
      {
         outputProcessors.get(i).setLowLevelControllerOutput(controllerRobotModel, lowLevelControllerCoreOutput);
      }
   }

   @Override
   public String toString()
   {
      StringBuffer buf = new StringBuffer();
      for (RobotOutputProcessor outputProcessor : outputProcessors)
      {
         buf.append(outputProcessor.getClass().getSimpleName() + "\n");
      }
      return buf.toString();
   }
}

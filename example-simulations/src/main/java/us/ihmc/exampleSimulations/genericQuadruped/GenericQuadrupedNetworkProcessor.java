package us.ihmc.exampleSimulations.genericQuadruped;

import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedPhysicalProperties;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedPointFootSnapperParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkModuleParameters;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;

public class GenericQuadrupedNetworkProcessor extends QuadrupedNetworkProcessor
{
   private static QuadrupedNetworkModuleParameters networkModuleParameters = new QuadrupedNetworkModuleParameters();

   static
   {
      networkModuleParameters.enableFootstepPlanningModule(true);
      networkModuleParameters.enableStepTeleopModule(true);
      networkModuleParameters.enableBodyTeleopModule(true);
      networkModuleParameters.enableBodyHeightTeleopModule(true);
   }

   public GenericQuadrupedNetworkProcessor(DomainFactory.PubSubImplementation pubSubImplementation)
   {
      this(new GenericQuadrupedModelFactory(),
           new GenericQuadrupedPhysicalProperties().getNominalBodyHeight(), new GenericQuadrupedXGaitSettings(),
           new GenericQuadrupedPointFootSnapperParameters(), pubSubImplementation);
   }

   public GenericQuadrupedNetworkProcessor(FullQuadrupedRobotModelFactory robotModel, double nominalHeight, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                           PointFootSnapperParameters pointFootSnapperParameters, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(robotModel, networkModuleParameters, nominalHeight, xGaitSettings, pointFootSnapperParameters, pubSubImplementation);
   }

   public static void main(String[] args)
   {
      new GenericQuadrupedNetworkProcessor(DomainFactory.PubSubImplementation.INTRAPROCESS);
   }
}

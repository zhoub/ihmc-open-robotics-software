package us.ihmc.valkyrie.pushRecovery;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCPushRecoveryWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
public class ValkyriePushRecoveryWalkingTest extends DRCPushRecoveryWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ValkyrieWalkingControllerParameters(getJointMap(), RobotTarget.SCS)
            {
               @Override
               public boolean useOptimizationBasedICPController()
               {
                  return false;
               }
            };
         }
      };
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @ContinuousIntegrationTest(estimatedDuration = 36.8)
   @Test
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(700.0);
      super.testPushLeftEarlySwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.4)
   @Test
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushLeftInitialTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 62.6)
   @Test
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightInitialTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 38.1)
   @Test
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightLateSwing();
   }

   @ContinuousIntegrationTest(estimatedDuration = 63.1)
   @Test
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setPushMagnitude(700.0);
      super.testPushRightThenLeftMidSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 37.9)
   @Test
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException
   {
      super.testPushRightTransferState();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 29.3, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheBack();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 37.7)
   @Test
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException
   {
      super.testPushTowardsTheFront();
   }
}
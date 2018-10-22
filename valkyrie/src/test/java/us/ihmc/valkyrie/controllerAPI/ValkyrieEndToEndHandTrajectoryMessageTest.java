package us.ihmc.valkyrie.controllerAPI;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyriePhysicalProperties;

public class ValkyrieEndToEndHandTrajectoryMessageTest extends EndToEndHandTrajectoryMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 46.3)
   @Test
   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlFrame();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 15.7)
   @Test
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      super.testMessageWithTooManyTrajectoryPoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 36.8, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testMultipleTrajectoryPoints() throws Exception
   {
      super.testMultipleTrajectoryPoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 65.8)
   @Test
   public void testQueuedMessages() throws Exception
   {
      super.testQueuedMessages();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 31.2)
   @Test
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      super.testQueueStoppedWithOverrideMessage();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 17.1, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testQueueWithWrongPreviousId() throws Exception
   {
      super.testQueueWithWrongPreviousId();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 42.0)
   @Test
   public void testSingleTrajectoryPoint() throws Exception
   {
      super.testSingleTrajectoryPoint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 43.9)
   @Test
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   public double getLegLength()
   {
      return ValkyriePhysicalProperties.getLegLength();
   }
}

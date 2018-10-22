package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndArmTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasEndToEndArmTrajectoryMessageTest extends EndToEndArmTrajectoryMessageTest
{

   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 16.5)
   @Test
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      super.testMessageWithTooManyTrajectoryPoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 36.0)
   @Test
   public void testMultipleTrajectoryPoints() throws Exception
   {
      super.testMultipleTrajectoryPoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.9, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testQueuedMessages() throws Exception
   {
      super.testQueuedMessages();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 19.0)
   @Test
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      super.testQueueStoppedWithOverrideMessage();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 16.9, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testQueueWithWrongPreviousId() throws Exception
   {
      super.testQueueWithWrongPreviousId();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 27.8, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testSingleTrajectoryPoint() throws Exception
   {
      super.testSingleTrajectoryPoint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 34.7)
   @Test
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }
}

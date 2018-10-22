package us.ihmc.valkyrie.controllerAPI;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndPelvisTrajectoryMessageTest extends EndToEndPelvisTrajectoryMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

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
   @ContinuousIntegrationTest(estimatedDuration = 68.6, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testHeightModeSwitchWhileWalking() throws Exception
   {
      super.testHeightModeSwitchWhileWalking();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 67.1, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testHeightUsingMultipleWaypoints() throws Exception
   {
      super.testHeightUsingMultipleWaypoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 73.9, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testHeightUsingMultipleWaypointsWhileWalking() throws Exception
   {
      super.testHeightUsingMultipleWaypointsWhileWalking();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 24.1, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testMultipleWaypoints() throws Exception
   {
      super.testMultipleWaypoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 33.1)
   @Test
   public void testSingleWaypoint() throws Exception
   {
      super.testSingleWaypoint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 76.4)
   @Test
   public void testSingleWaypointAndWalk() throws Exception
   {
      super.testSingleWaypointAndWalk();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 31.5)
   @Test
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 91.2)
   @Test
   public void testSingleWaypointThenManualChange() throws Exception
   {
      super.testSingleWaypointThenManualChange();
   }

}

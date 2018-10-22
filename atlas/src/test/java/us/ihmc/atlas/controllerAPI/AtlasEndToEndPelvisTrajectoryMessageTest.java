package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;

public class AtlasEndToEndPelvisTrajectoryMessageTest extends EndToEndPelvisTrajectoryMessageTest
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

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test
   public void testSixDoFMovementsOfPelvis() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      super.testSixDoFMovementsOfPelvis();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.1)
   @Test
   public void testSingleWaypoint() throws Exception
   {
      super.testSingleWaypoint();
   }

   @ContinuousIntegrationTest(estimatedDuration = 29.1)
   @Test
   public void testSingleWaypointAndAbort() throws Exception
   {
      super.testSingleWaypointAndAbort();
   }

   @ContinuousIntegrationTest(estimatedDuration = 75.8)
   @Test
   public void testSingleWaypointAndWalk() throws Exception
   {
      super.testSingleWaypointAndWalk();
   }

   @ContinuousIntegrationTest(estimatedDuration = 22.1, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testMultipleWaypoints() throws Exception
   {
      super.testMultipleWaypoints();
   }

   @ContinuousIntegrationTest(estimatedDuration = 29.3)
   @Test
   @SuppressWarnings("unchecked")
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 67.1, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testHeightUsingMultipleWaypoints() throws Exception
   {
      super.testHeightUsingMultipleWaypoints();
   }

   @ContinuousIntegrationTest(estimatedDuration = 72.0)
   @Test
   public void testHeightUsingMultipleWaypointsWhileWalking() throws Exception
   {
      super.testHeightUsingMultipleWaypointsWhileWalking();
   }

   @ContinuousIntegrationTest(estimatedDuration = 61.1, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testHeightModeSwitchWhileWalking() throws Exception
   {
      super.testHeightModeSwitchWhileWalking();
   }


   @ContinuousIntegrationTest(estimatedDuration = 91.2, categoriesOverride = {IntegrationCategory.EXCLUDE})
   @Test
   public void testSingleWaypointThenManualChange() throws Exception
   {
      super.testSingleWaypointThenManualChange();
   }
}

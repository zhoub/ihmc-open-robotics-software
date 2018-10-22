package us.ihmc.atlas.ObstacleCourseTests;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import static us.ihmc.continuousIntegration.IntegrationCategory.SLOW;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
public class AtlasObstacleCourseFlatTest extends DRCObstacleCourseFlatTest
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
   @ContinuousIntegrationTest(estimatedDuration = 76.7, categoriesOverride = SLOW)
   @Test
   public void testACoupleMoreQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      super.testACoupleMoreQueuedControllerCommands();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 78.7)
   @Test
   public void testACoupleQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      super.testACoupleQueuedControllerCommands();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 59.7)
   @Test
   public void testACoupleStepsUsingQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      super.testACoupleStepsUsingQueuedControllerCommands();
   }

   @Override
   // Invoked manually to test memory & thread leaks
   @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.MANUAL)
   @Test
   public void testForMemoryLeaks() throws Exception
   {
      super.testForMemoryLeaks();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 113.3)
   @Test
   public void testLongStepsMaxHeightPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      super.testLongStepsMaxHeightPauseAndResume();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 54.2)
   @Test
   public void testRotatedStepInTheAir() throws SimulationExceededMaximumTimeException
   {
      super.testRotatedStepInTheAir();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 85.5, categoriesOverride = SLOW)
   @Test
   public void testSimpleScripts() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSimpleScripts();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 26.1, categoriesOverride = SLOW)
   @Test
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      super.testStandingForACoupleSeconds();
   }

   @Override
   // TODO re-enable that test when we have polygon to polygon contact model for SCS
   @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test
   public void testStandingOnUnevenTerrainForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      super.testStandingOnUnevenTerrainForACoupleSeconds();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 34.0)
   @Test
   public void testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly() throws SimulationExceededMaximumTimeException
   {
      super.testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 96.1)
   @Test
   public void testTurningInPlaceAndPassingPI() throws SimulationExceededMaximumTimeException
   {
      super.testTurningInPlaceAndPassingPI();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 68.6)
   @Test
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 83.3)
   @Test
   public void testWalkingUpToRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithShortSteps();
   }
}

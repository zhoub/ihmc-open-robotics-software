package us.ihmc.atlas;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCFlatGroundRewindabilityTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Tag("flaky")
public class AtlasFlatGroundRewindabilityTest extends DRCFlatGroundRewindabilityTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test // timeout = 520000
   public void testCanRewindAndGoForward() throws UnreasonableAccelerationException
   {
      super.testCanRewindAndGoForward();
   }

   @Override
   @Disabled
   @Test // timeout = 520000
   public void testRewindabilityWithSimpleFastMethod() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      super.testRewindabilityWithSimpleFastMethod();
   }

   @Override
   // This takes a long time. Use it for debugging where the broken changes were made when the tests above fail.
   @Disabled
   @Test // timeout = 520000
   public void testRewindabilityWithSlowerMoreExtensiveMethod() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      super.testRewindabilityWithSlowerMoreExtensiveMethod();
   }

   @Override
   @Test // timeout = 520000
   public void testRunsTheSameWayTwice() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testRunsTheSameWayTwice();
   }

}

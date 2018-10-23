package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCHeadTrajectoryBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasHeadTrajectoryBehaviorTest extends DRCHeadTrajectoryBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasHeadTrajectoryBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   }

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
   @Test // timeout = 370000
   public void testHeadPitch() throws SimulationExceededMaximumTimeException
   {
      super.testHeadPitch();
   }

   @Override
   @Disabled
   @Test // timeout = 950000
   public void testHeadRoll() throws SimulationExceededMaximumTimeException
   {
      super.testHeadRoll();
   }

   @Override
   @Disabled
   @Test // timeout = 950000
   public void testHeadYaw() throws SimulationExceededMaximumTimeException
   {
      super.testHeadYaw();
   }

   @Override
   @Disabled
   @Test // timeout = 950000
   public void testRandomOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testRandomOrientation();
   }

}

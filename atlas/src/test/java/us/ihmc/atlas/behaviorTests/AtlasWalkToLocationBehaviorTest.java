package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCWalkToLocationBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasWalkToLocationBehaviorTest extends DRCWalkToLocationBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasWalkToLocationBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
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
   @Tag("slow")
   @Test
   public void testTurn361DegreesInPlace() throws SimulationExceededMaximumTimeException
   {
      super.testTurn361DegreesInPlace();
   }
   
   @Override
   @Test
   public void testWalkAndStopBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAndStopBehavior();
   }
   
   @Override
   @Test
   public void testWalkAtAngleAndFinishAlignedWithInitialOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleAndFinishAlignedWithInitialOrientation();
   }
   
   @Override
   @Tag("slow")
   @Test
   public void testWalkAtAngleAndFinishAlignedWithWalkingPath() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleAndFinishAlignedWithWalkingPath();
   }
   
   @Override
   @Tag("slow")
   @Test
   public void testWalkAtAngleUsingStartOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleUsingStartOrientation();
   }
   
   @Override
   @Test
   public void testWalkAtAngleUsingStartTargetMeanOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleUsingStartTargetMeanOrientation();
   }
   
   @Override
   @Tag("slow")
   @Test
   public void testWalkAtAngleUsingTargetOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkAtAngleUsingTargetOrientation();
   }
   
   @Override
   @Test
   public void testWalkBackwardsASmallAmountWithoutTurningInPlace() throws SimulationExceededMaximumTimeException
   {
      super.testWalkBackwardsASmallAmountWithoutTurningInPlace();
   }
   
   @Override
   @Test
   public void testWalkForwardsX() throws SimulationExceededMaximumTimeException
   {
      super.testWalkForwardsX();
   }
   
   @Override
   @Disabled
   @Test
   public void testWalkPauseAndResumeBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testWalkPauseAndResumeBehavior();
   }
   
   @Override
   @Disabled
   @Test
   public void testWalkPauseAndResumeOnLastStepBehavior() throws SimulationExceededMaximumTimeException
   {
      super.testWalkPauseAndResumeOnLastStepBehavior();
   }
   
   @Override
   @Disabled
   @Test
   public void testWalkStopAndWalkToDifferentLocation() throws SimulationExceededMaximumTimeException
   {
      super.testWalkStopAndWalkToDifferentLocation();
   }
}

package us.ihmc.valkyrie.controllerAPI;

import org.junit.Test;
import us.ihmc.avatar.controllerAPI.EndToEndEndFootBearingMessageTest;
import us.ihmc.avatar.controllerAPI.EndToEndHandLoadBearingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndHandLoadBearingTest extends EndToEndHandLoadBearingTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false, true);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   public double getDesiredPelvisHeight()
   {
      return 0.875;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 53.7)
   @Test(timeout = 270000)
   public void testUsingHand() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      super.testUsingHand();
   }
}

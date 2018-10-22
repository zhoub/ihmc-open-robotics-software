package us.ihmc.exampleSimulations.genericQuadruped.controller.position;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlTurningVelocityTest;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Disabled
public class GenericQuadrupedPositionCrawlTurningVelocityTest extends QuadrupedPositionCrawlTurningVelocityTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 150.0)
   @Test
   public void testTurnInPlaceRegularSpeed() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testTurnInPlaceRegularSpeed();
   }
   
   //"Turn in place slowly still fails due to CoM shifting outside support polygon. Need to fix it..."
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 150.0)
   @Test
   public void testTurnInPlaceSlowly() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testTurnInPlaceSlowly();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 500.0)
   @Test
   public void testWalkingBackwardStoppingAndTurning() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardStoppingAndTurning();
   }
}

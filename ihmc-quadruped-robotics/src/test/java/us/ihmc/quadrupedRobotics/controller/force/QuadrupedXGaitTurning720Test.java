package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class QuadrupedXGaitTurning720Test implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @Before
   public void setup()
   {
      try
      {
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 370.6)
   @Test(timeout = 1900000)
   public void rotate720InPlaceRight() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setStanceWidth(0.35);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, -0.5);

      int numSpins = 2;
      for (int i = 0; i < numSpins; i++)
      {
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 10.0));
         conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI / 4.0, 1e-2));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI / 2.0, 1e-2));
         conductor.simulate();

         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 10.0));
         conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI / 4.0, 1e-2));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), 0.0, 1e-2));
         conductor.simulate();
      }

      conductor.concludeTesting();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 370.6)
   @Test(timeout = 1900000)
   public void rotate720InPlaceLeft() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setStanceWidth(0.35);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.5);

      int numSpins = 2;
      for (int i = 0; i < numSpins; i++)
      {
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 10.0));
         conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI / 4.0, 1e-2));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI / 2.0, 1e-2));
         conductor.simulate();

         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addSustainGoal(YoVariableTestGoal.doubleLessThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 10.0));
         conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI / 4.0, 1e-2));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), 0.0, 1e-2));
         conductor.simulate();
      }

      conductor.concludeTesting();
   }
}

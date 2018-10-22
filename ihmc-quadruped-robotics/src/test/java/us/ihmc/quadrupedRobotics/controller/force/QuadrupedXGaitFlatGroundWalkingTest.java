package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import org.junit.jupiter.api.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedXGaitFlatGroundWalkingTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   public abstract double getPacingWidth();

   public abstract double getFastWalkingSpeed();
   public abstract double getSlowWalkingSpeed();
   public abstract double getWalkingAngularVelocity();
   public abstract double getWalkingSpeedWhileTurning();

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }
   
   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testWalkingForwardFast()
   {
      testFlatGroundWalking(90.0, getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testWalkingForwardSlow()
   {
      testFlatGroundWalking(90.0, getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testWalkingBackwardsFast()
   {
      testFlatGroundWalking(90.0, -getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testWalkingBackwardsSlow()
   {
      testFlatGroundWalking(90.0, -getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testWalkingInAForwardLeftCircle()
   {
      testWalkingInASemiCircle(90.0, getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testWalkingInAForwardRightCircle()
   {
      testWalkingInASemiCircle(90.0, getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testWalkingInABackwardLeftCircle()
   {
      testWalkingInASemiCircle(90.0, -getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testWalkingInABackwardRightCircle()
   {
      testWalkingInASemiCircle(90.0, -getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }


   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testTrottingForwardFast()
   {
      testFlatGroundWalking(180.0, getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testTrottingForwardSlow()
   {
      testFlatGroundWalking(180.0, getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testTrottingBackwardsFast()
   {
      testFlatGroundWalking(180.0, -getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testTrottingBackwardsSlow()
   {
      testFlatGroundWalking(180.0, -getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testTrottingInAForwardLeftCircle()
   {
      testWalkingInASemiCircle(180.0, getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testTrottingInAForwardRightCircle()
   {
      testWalkingInASemiCircle(180.0, getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testTrottingInABackwardLeftCircle()
   {
      testWalkingInASemiCircle(180.0, -getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testTrottingInABackwardRightCircle()
   {
      testWalkingInASemiCircle(180.0, -getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   private void testFlatGroundWalking(double endPhaseShift, double walkingSpeed)
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(endPhaseShift);

      double walkTime = 5.0;
      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));

      double finalPositionX = walkTime * walkingSpeed * 0.7;
      if(walkingSpeed > 0.0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), finalPositionX));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), finalPositionX));
      }

      conductor.simulate();
   }


   private void testWalkingInASemiCircle(double endPhaseShift, double walkingSpeed, double angularVelocity)
   {
      stepTeleopManager.setShiftPlanBasedOnStepAdjustment(false);
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      double radius = Math.abs(walkingSpeed / angularVelocity);
      double expectedSemiCircleWalkTime = Math.PI / Math.abs(angularVelocity);

      stepTeleopManager.requestXGait();
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(endPhaseShift);
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, angularVelocity);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), expectedSemiCircleWalkTime * 1.5);
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.signum(angularVelocity) * Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.2));
      conductor.addTerminalGoal(YoVariableTestGoal.or(
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI, 0.2),
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI, 0.2)));

      if(Math.signum(walkingSpeed) > 0.0)
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), radius * walkingSpeed * 0.6));
      }
      else
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), radius * walkingSpeed * 0.6));
      }

      conductor.simulate();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testPacingForwardFast()
   {
      testFlatGroundPacing(getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testPacingForwardSlow()
   {
      testFlatGroundPacing(getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testPacingBackwardsFast()
   {
      testFlatGroundPacing(-getFastWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testPacingBackwardsSlow()
   {
      testFlatGroundPacing(-getSlowWalkingSpeed());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testPacingInAForwardLeftCircle()
   {
      testPacingInASemiCircle(getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 246.9)
   @Test
   public void testPacingInAForwardRightCircle()
   {
      testPacingInASemiCircle(getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testPacingInABackwardLeftCircle()
   {
      testPacingInASemiCircle(-getWalkingSpeedWhileTurning(), -getWalkingAngularVelocity());
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test
   public void testPacingInABackwardRightCircle()
   {
      testPacingInASemiCircle(-getWalkingSpeedWhileTurning(), getWalkingAngularVelocity());
   }


   private void testFlatGroundPacing(double walkingSpeed)
   {
      stepTeleopManager.getXGaitSettings().setStanceWidth(getPacingWidth());

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(0.0);

      double walkTime = 5.0;
      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));

      double finalPositionX = walkTime * walkingSpeed * 0.7;
      if(walkingSpeed > 0.0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), finalPositionX));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), finalPositionX));
      }

      conductor.simulate();
   }

   private void testPacingInASemiCircle(double walkingSpeed, double angularVelocity)
   {
      stepTeleopManager.getXGaitSettings().setStanceWidth(getPacingWidth());

      stepTeleopManager.setShiftPlanBasedOnStepAdjustment(false);
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      double radius = Math.abs(walkingSpeed / angularVelocity);
      double expectedSemiCircleWalkTime = Math.PI / Math.abs(angularVelocity);

      stepTeleopManager.requestXGait();
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(0.0);
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, angularVelocity);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), expectedSemiCircleWalkTime * 1.5);
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.signum(angularVelocity) * Math.PI / 2, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 0.0, 0.2));
      conductor.addTerminalGoal(YoVariableTestGoal.or(
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI, 0.2),
            YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), Math.PI, 0.2)));

      if(Math.signum(walkingSpeed) > 0.0)
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), radius * walkingSpeed * 0.6));
      }
      else
      {
         conductor.addWaypointGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), radius * walkingSpeed * 0.6));
      }

      conductor.simulate();
   }
}

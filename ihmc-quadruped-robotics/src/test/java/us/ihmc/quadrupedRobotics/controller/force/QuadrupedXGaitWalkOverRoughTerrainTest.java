package us.ihmc.quadrupedRobotics.controller.force;

import static junit.framework.TestCase.assertTrue;

import java.io.IOException;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.DefaultPointFootSnapperParameters;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.PlanarRegionBasedPointFootSnapper;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.CinderBlockFieldPlanarRegionEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.PlanarRegionEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.SingleStepEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.StaircaseEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.VaryingHeightTiledGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.ZigZagSlopeEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedXGaitWalkOverRoughTerrainTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   public abstract QuadrupedXGaitSettingsReadOnly getXGaitSettings();

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
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

   @Test
   public void testWalkingOverTiledGround() throws IOException
   {
      VaryingHeightTiledGroundEnvironment environment = new VaryingHeightTiledGroundEnvironment(0.75, 10, 4, -0.1, 0.1);
      double walkTime = 12.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 2.0;

      runWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, getXGaitSettings());
   }

   @Test
   public void testWalkingOverSingleStepUp() throws IOException
   {
      SingleStepEnvironment environment = new SingleStepEnvironment(0.1, 1.0);
      double walkTime = 12.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 2.0;

      runWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, getXGaitSettings());
   }

   @Test
   public void testWalkingOverConsecutiveRamps() throws IOException
   {
      ZigZagSlopeEnvironment environment = new ZigZagSlopeEnvironment(0.15, 0.5, 20, -0.1);
      double walkTime = 5.0;
      double walkingSpeed = 0.25;
      double minimumXPositionAfterWalking = 3.0;

      runWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, getXGaitSettings());
   }

   @Test
   public void testWalkingOverCinderBlockField() throws IOException
   {
      CinderBlockFieldPlanarRegionEnvironment environment = new CinderBlockFieldPlanarRegionEnvironment();
      double walkTime = 40.0;
      double walkingSpeed = 0.3;
      double minimumXPositionAfterWalking = 8.0;

      runWalkingOverTerrain(environment, walkTime, walkingSpeed, minimumXPositionAfterWalking, getXGaitSettings());
   }

   @Test
   public void testWalkingUpStaircase() throws IOException
   {
      double stepHeight = 0.13;
      double stepLength = 0.8;
      int numberOfSteps = 6;
      StaircaseEnvironment staircaseEnvironment = new StaircaseEnvironment(numberOfSteps, stepHeight, stepLength);
      double walkTime = 20.0;
      double walkingSpeed = 0.3;
      double minimumXPositionAfterWalking  = numberOfSteps * stepLength + 0.5;

      runWalkingOverTerrain(staircaseEnvironment, walkTime, walkingSpeed, minimumXPositionAfterWalking, getXGaitSettings());
   }

   private void runWalkingOverTerrain(PlanarRegionEnvironmentInterface environment, double walkTime, double walkingSpeed,
                                      double minimumXPositionAfterWalking, QuadrupedXGaitSettingsReadOnly xGaitSettings) throws IOException
   {
      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromSystemProperties();
      simulationConstructionSetParameters.setUseAutoGroundGraphics(false);

      quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setScsParameters(simulationConstructionSetParameters);
      quadrupedTestFactory.setTerrainObject3D(environment.getTerrainObject3D());
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);

      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      PlanarRegionBasedPointFootSnapper snapper = new PlanarRegionBasedPointFootSnapper(new DefaultPointFootSnapperParameters());
      snapper.setPlanarRegionsList(environment.getPlanarRegionsList());
      stepTeleopManager.setStepSnapper(snapper);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.getXGaitSettings().set(xGaitSettings);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));

      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), minimumXPositionAfterWalking));
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));
      conductor.simulate();


      stepTeleopManager.requestStanding();
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 2.0));
      conductor.simulate();

      assertTrue(stepTeleopManager.isInStandState());
   }
}
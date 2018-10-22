package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import static us.ihmc.continuousIntegration.IntegrationCategory.SLOW;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedSquaredUpInitialPosition;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceBasedStandControllerTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedForceBasedStandControllerTest extends QuadrupedForceBasedStandControllerTest
{
   public double getTranslationShift()
   {
      return 0.05;
   }

   public double getTranslationDelta()
   {
      return 0.01;
   }

   public double getOrientationShift()
   {
      return Math.toRadians(5.0);
   }

   public double getOrientationDelta()
   {
      return Math.toRadians(1.0);
   }

   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      GenericQuadrupedTestFactory testFactory = new GenericQuadrupedTestFactory();
      testFactory.setInitialPosition(new GenericQuadrupedSquaredUpInitialPosition());

      return testFactory;
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0, categoriesOverride = SLOW)
   @Test
   public void testStandingAndResistingPushesOnFrontRightHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnFrontRightHipRoll();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0, categoriesOverride = SLOW)
   @Test
   public void testStandingAndResistingPushesOnHindLeftHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnHindLeftHipRoll();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0, categoriesOverride = SLOW)
   @Test
   public void testStandingAndResistingPushesOnHindRightHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnHindRightHipRoll();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0, categoriesOverride = SLOW)
   @Test
   public void testStandingAndResistingPushesOnFrontLeftHipRoll() throws IOException
   {
      super.testStandingAndResistingPushesOnFrontLeftHipRoll();
   }

   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test
   public void testStandingAndResistingPushesOnBody() throws IOException
   {
      super.testStandingAndResistingPushesOnBody();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testStandingUpAndAdjustingCoM() throws IOException
   {
      super.testStandingUpAndAdjustingCoM();
   }
}

package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitPushRecoveryTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedXGaitPushRecoveryTest extends QuadrupedXGaitPushRecoveryTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Override
   public double getWalkingSpeed()
   {
      return 0.8;
   }

   @Override
   public double getStepDuration()
   {
      return 0.35;
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test
   public void testWalkingForwardFastWithPush()
   {
      super.testWalkingForwardFastWithPush();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testScriptedWalkingForwardFastWithPush()
   {
      super.testScriptedWalkingForwardFastWithPush();
   }

}

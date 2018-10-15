package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundWalkingTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GenericQuadrupedXGaitFlatGroundWalkingTest extends QuadrupedXGaitFlatGroundWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Override
   public double getPacingWidth()
   {
      return 0.2;
   }

   @Override
   public double getFastWalkingSpeed()
   {
      return 0.8;
   }

   @Override
   public double getSlowWalkingSpeed()
   {
      return 0.1;
   }

   @Override
   public double getWalkingAngularVelocity()
   {
      return 0.3;
   }

   @Override
   public double getWalkingSpeedWhileTurning()
   {
      return 0.6;
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testWalkingForwardFast()
   {
      super.testWalkingForwardFast();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 720000)
   public void testWalkingForwardSlow()
   {
      super.testWalkingForwardSlow();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 460000)
   public void testWalkingBackwardsFast()
   {
      super.testWalkingBackwardsFast();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 670000)
   public void testWalkingBackwardsSlow()
   {
      super.testWalkingBackwardsSlow();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 1100000)
   public void testWalkingInAForwardLeftCircle()
   {
      super.testWalkingInAForwardLeftCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testWalkingInAForwardRightCircle()
   {
      super.testWalkingInAForwardRightCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testWalkingInABackwardLeftCircle()
   {
      super.testWalkingInABackwardLeftCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 1500000)
   public void testWalkingInABackwardRightCircle()
   {
      super.testWalkingInABackwardRightCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testTrottingForwardFast()
   {
      super.testTrottingForwardFast();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 720000)
   public void testTrottingForwardSlow()
   {
      super.testTrottingForwardSlow();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 460000)
   public void testTrottingBackwardsFast()
   {
      super.testTrottingBackwardsFast();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 670000)
   public void testTrottingBackwardsSlow()
   {
      super.testTrottingBackwardsSlow();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1100000)
   public void testTrottingInAForwardLeftCircle()
   {
      super.testTrottingInAForwardLeftCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 1200000)
   public void testTrottingInAForwardRightCircle()
   {
      super.testTrottingInAForwardRightCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 1200000)
   public void testTrottingInABackwardLeftCircle()
   {
      super.testTrottingInABackwardLeftCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1500000)
   public void testTrottingInABackwardRightCircle()
   {
      super.testTrottingInABackwardRightCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 630000)
   public void testPacingForwardFast()
   {
      super.testPacingForwardFast();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 720000)
   public void testPacingForwardSlow()
   {
      super.testPacingForwardSlow();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 460000)
   public void testPacingBackwardsFast()
   {
      super.testPacingBackwardsFast();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 670000)
   public void testPacingBackwardsSlow()
   {
      super.testPacingBackwardsSlow();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1100000)
   public void testPacingInAForwardLeftCircle()
   {
      super.testPacingInAForwardLeftCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 246.9, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 1200000)
   public void testPacingInAForwardRightCircle()
   {
      super.testPacingInAForwardRightCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0)
   @Test(timeout = 1200000)
   public void testPacingInABackwardLeftCircle()
   {
      super.testPacingInABackwardLeftCircle();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 1500000)
   public void testPacingInABackwardRightCircle()
   {
      super.testPacingInABackwardRightCircle();
   }

}

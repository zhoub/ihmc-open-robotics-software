package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitFlatGroundWalkingTest;

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

   @Test
   public void testWalkingForwardFast()
   {
      super.testWalkingForwardFast();
   }

   @Tag("slow")
   @Test
   public void testWalkingForwardSlow()
   {
      super.testWalkingForwardSlow();
   }

   @Test
   public void testWalkingBackwardsFast()
   {
      super.testWalkingBackwardsFast();
   }

   @Tag("slow")
   @Test
   public void testWalkingBackwardsSlow()
   {
      super.testWalkingBackwardsSlow();
   }

   @Tag("slow")
   @Test
   public void testWalkingInAForwardLeftCircle()
   {
      super.testWalkingInAForwardLeftCircle();
   }

   @Test
   public void testWalkingInAForwardRightCircle()
   {
      super.testWalkingInAForwardRightCircle();
   }

   @Test
   public void testWalkingInABackwardLeftCircle()
   {
      super.testWalkingInABackwardLeftCircle();
   }

   @Tag("slow")
   @Test
   public void testWalkingInABackwardRightCircle()
   {
      super.testWalkingInABackwardRightCircle();
   }

   @Test
   public void testTrottingForwardFast()
   {
      super.testTrottingForwardFast();
   }

   @Tag("slow")
   @Test
   public void testTrottingForwardSlow()
   {
      super.testTrottingForwardSlow();
   }

   @Test
   public void testTrottingBackwardsFast()
   {
      super.testTrottingBackwardsFast();
   }

   @Tag("slow")
   @Test
   public void testTrottingBackwardsSlow()
   {
      super.testTrottingBackwardsSlow();
   }

   @Test
   public void testTrottingInAForwardLeftCircle()
   {
      super.testTrottingInAForwardLeftCircle();
   }

   @Tag("slow")
   @Test
   public void testTrottingInAForwardRightCircle()
   {
      super.testTrottingInAForwardRightCircle();
   }

   @Tag("slow")
   @Test
   public void testTrottingInABackwardLeftCircle()
   {
      super.testTrottingInABackwardLeftCircle();
   }

   @Test
   public void testTrottingInABackwardRightCircle()
   {
      super.testTrottingInABackwardRightCircle();
   }

   @Test
   public void testPacingForwardFast()
   {
      super.testPacingForwardFast();
   }

   @Tag("slow")
   @Test
   public void testPacingForwardSlow()
   {
      super.testPacingForwardSlow();
   }

   @Test
   public void testPacingBackwardsFast()
   {
      super.testPacingBackwardsFast();
   }

   @Test
   public void testPacingBackwardsSlow()
   {
      super.testPacingBackwardsSlow();
   }

   @Test
   public void testPacingInAForwardLeftCircle()
   {
      super.testPacingInAForwardLeftCircle();
   }

   @Tag("slow")
   @Test
   public void testPacingInAForwardRightCircle()
   {
      super.testPacingInAForwardRightCircle();
   }

   @Test
   public void testPacingInABackwardLeftCircle()
   {
      super.testPacingInABackwardLeftCircle();
   }

   @Tag("slow")
   @Test
   public void testPacingInABackwardRightCircle()
   {
      super.testPacingInABackwardRightCircle();
   }

}

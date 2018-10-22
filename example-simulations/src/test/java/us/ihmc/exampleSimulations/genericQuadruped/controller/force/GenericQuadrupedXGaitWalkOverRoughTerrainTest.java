package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import java.io.IOException;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import junit.framework.AssertionFailedError;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitWalkOverRoughTerrainTest;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;

public class GenericQuadrupedXGaitWalkOverRoughTerrainTest extends QuadrupedXGaitWalkOverRoughTerrainTest
{
   private QuadrupedXGaitSettingsReadOnly xGaitSettings;


   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test
   public void testWalkingUpStaircase() throws IOException
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingUpStaircase();
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0)
   @Test
   public void testWalkingOverTiledGround() throws IOException, AssertionFailedError
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverTiledGround();
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testWalkingOverSingleStepUp() throws IOException, AssertionFailedError
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverSingleStepUp();
   }

   @ContinuousIntegrationTest(estimatedDuration = 80.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test
   public void testWalkingOverConsecutiveRamps() throws IOException, AssertionFailedError
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverConsecutiveRamps();
   }

   @Disabled
   @Test
   public void testWalkingOverCinderBlockField() throws IOException, AssertionFailedError
   {
      xGaitSettings = new GenericQuadrupedXGaitSettings();
      super.testWalkingOverCinderBlockField();
   }

   @Override
   public QuadrupedXGaitSettingsReadOnly getXGaitSettings()
   {
      return xGaitSettings;
   }

   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
}

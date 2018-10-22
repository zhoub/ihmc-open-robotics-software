package us.ihmc.manipulation;

import org.junit.jupiter.api.Tag;
import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.continuousIntegration.ContinuousIntegrationSuite;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@Tag("fast")
@SuiteClasses
({
   us.ihmc.manipulation.planning.gradientDescent.GradientDescentTest.class, 
   us.ihmc.manipulation.planning.manifold.ReachingManifoldVisualizingTest.class
 })

public class IHMCManipulationTests
{
   public static void main(String[] args)
   {

   }
}
package us.ihmc.robotics.hyperCubeTree;

import static org.junit.Assert.assertEquals;

import org.junit.jupiter.api.Test;

public class OneDimensionalBoundsTest
{
   private static final double eps = 1e-14;

	@Test
   public void testIntersection()
   {
      OneDimensionalBounds zeroToOne = new OneDimensionalBounds(0.0, 1.0);
      OneDimensionalBounds zeroToTwenty = new OneDimensionalBounds(0.0, 20.0);
      OneDimensionalBounds result = zeroToOne.intersectionWith(zeroToTwenty);
      assertEquals(1.0,result.max(),eps);
   }

}

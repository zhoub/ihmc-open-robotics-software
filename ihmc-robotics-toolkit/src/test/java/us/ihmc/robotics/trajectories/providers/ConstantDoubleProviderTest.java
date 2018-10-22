package us.ihmc.robotics.trajectories.providers;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;

public class ConstantDoubleProviderTest
{

   private static final double EPS = 1e-12;

	@Test
   public void test()
   {
      Random random = new Random();
      double expectedValue = RandomNumbers.nextDouble(random, Double.MIN_VALUE, Double.MAX_VALUE);
      ConstantDoubleProvider constantDoubleProvider = new ConstantDoubleProvider(expectedValue);
      double actualValue = constantDoubleProvider.getValue();
      
      assertEquals(expectedValue, actualValue, EPS);
   }

}

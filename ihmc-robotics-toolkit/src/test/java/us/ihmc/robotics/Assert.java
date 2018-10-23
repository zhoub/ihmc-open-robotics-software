package us.ihmc.robotics;

public class Assert
{
   static public void assertTrue(String message, boolean condition)
   {
   }

   static public void assertTrue(boolean condition)
   {
      org.junit.jupiter.api.Assertions.assertTrue(condition);
   }

   static public void assertFalse(String message, boolean condition)
   {
      assertTrue(message, !condition);
   }

   static public void assertFalse(boolean condition)
   {
      assertFalse(null, condition);
   }

   static public void fail(String message)
   {
   }

   static public void fail()
   {
   }

   static public void assertEquals(String message, Object expected, Object actual)
   {
   }

   static public void assertEquals(Object expected, Object actual)
   {
   }

   static public void assertNotEquals(long first, long second)
   {
   }

   static public void assertNotEquals(String message, double first, double second, double delta)
   {
   }

   static public void assertNotEquals(double first, double second, double delta)
   {
   }

   public static void assertArrayEquals(Object[] expecteds, Object[] actuals)
   {
   }

   public static void assertArrayEquals(int[] expecteds, int[] actuals)
   {
   }

   public static void assertArrayEquals(double[] expecteds, double[] actuals, double delta)
   {
   }

   public static void assertArrayEquals(float[] expecteds, float[] actuals, float delta)
   {
   }

   static public void assertEquals(String message, double expected, double actual, double delta)
   {
   }

   static public void assertEquals(String message, float expected, float actual, float delta)
   {
   }

   static public void assertEquals(long expected, long actual)
   {
   }

   static public void assertEquals(String message, long expected, long actual)
   {
   }

   static public void assertEquals(double expected, double actual, double delta)
   {
   }

   static public void assertEquals(float expected, float actual, float delta)
   {
   }

   static public void assertNotNull(String message, Object object)
   {
   }

   static public void assertNotNull(Object object)
   {
   }

   static public void assertNull(String message, Object object)
   {
   }

   static public void assertNull(Object object)
   {
   }

   static public void assertSame(Object expected, Object actual)
   {
   }

   static public void assertNotSame(Object unexpected, Object actual)
   {
   }

   public static void assertArrayEquals(String string, double[] data, double[] ds, double d)
   {
   }
}

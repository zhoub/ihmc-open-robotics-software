package us.ihmc.tools.reflect;

import static org.junit.Assert.assertEquals;

import java.lang.reflect.Field;

import org.junit.After;
import org.junit.Before;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class ReflectionToolsTest
{

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@Test // timeout = 30000
   public final void testPrintDeclaredFields() throws SecurityException, NoSuchFieldException, IllegalArgumentException, IllegalAccessException
   {
      Double testDouble = 1.234;

      Field declaredField = Double.class.getDeclaredField("value");
      declaredField.setAccessible(true);
      String result = ReflectionTools.getStringRepresentationOfFieldContent(testDouble, declaredField);

      assertEquals("1.234", result);

      Float testFloat = 5.678f;

      declaredField = Float.class.getDeclaredField("value");
      declaredField.setAccessible(true);
      result = ReflectionTools.getStringRepresentationOfFieldContent(testFloat, declaredField);

      assertEquals("5.678", result);
   }

}

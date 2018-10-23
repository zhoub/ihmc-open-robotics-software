package us.ihmc.tools.string;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class StringAndRegularExpressionMatcherTest
{

	@Test // timeout = 30000
   public void testSimpleExactMatch()
   {
      StringAndRegularExpressionMatcher matcher = new StringAndRegularExpressionMatcher();

      matcher.addExactStringToMatch("HelloWorld");
      assertTrue(matcher.matches("HelloWorld"));
      assertFalse(matcher.matches("helloworld"));
      assertFalse(matcher.matches("banana"));
   }

	@Test // timeout = 30000
   public void testCaseInsensitiveExactMatch()
   {
      StringAndRegularExpressionMatcher matcher = new StringAndRegularExpressionMatcher(false);

      matcher.addExactStringToMatch("HelloWorld");
      assertTrue(matcher.matches("HelloWorld"));
      assertTrue(matcher.matches("helloworld"));
      assertFalse(matcher.matches("banana"));
   }

	@Test // timeout = 30000
   public void testRegularExpression()
   {
      StringAndRegularExpressionMatcher matcher = new StringAndRegularExpressionMatcher();

      matcher.addRegularExpression("HelloWorld");
      matcher.addRegularExpression("GoodbyeWorld");
      assertTrue(matcher.matches("HelloWorld"));
      assertFalse(matcher.matches("helloworld"));
      assertFalse(matcher.matches("banana"));

      matcher.addRegularExpression("b.*a");
      assertTrue(matcher.matches("banana"));
      assertFalse(matcher.matches("helloworld"));
      matcher.addRegularExpression("h.*w.*");
      assertTrue(matcher.matches("helloworld"));
   }

	@Test // timeout = 30000
   public void testRegularExpressionsAndStrings()
   {
      StringAndRegularExpressionMatcher matcher = new StringAndRegularExpressionMatcher();

      matcher.addExactStringToMatch("HelloWorld");
      matcher.addExactStringToMatch("GoodbyeWorld");
      assertTrue(matcher.matches("HelloWorld"));
      assertFalse(matcher.matches("helloworld"));
      assertFalse(matcher.matches("banana"));

      matcher.addRegularExpression("b.*a");
      assertTrue(matcher.matches("banana"));
      assertFalse(matcher.matches("helloworld"));
      matcher.addRegularExpression("h.*w.*");
      assertTrue(matcher.matches("helloworld"));
   }
}

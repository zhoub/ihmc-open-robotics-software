package us.ihmc.tools.exceptions;

import static org.junit.Assert.assertTrue;

import org.junit.jupiter.api.Test;

public class NoConvergenceExceptionTest
{
   @Test
   public void testCreateAndThrowSomeNoConvergenceExceptions()
   {
      int iter = 0;
      
      try
      {
         throw new NoConvergenceException();
      }
      catch (NoConvergenceException e)
      {
         iter = e.getIter();
      }
      
      assertTrue("Iter not equal -1", iter == -1);
      
      try
      {
         throw new NoConvergenceException(5);
      }
      catch (NoConvergenceException e)
      {
         iter = e.getIter();
      }
      
      assertTrue("Iter not equal -1", iter == 5);
   }
}

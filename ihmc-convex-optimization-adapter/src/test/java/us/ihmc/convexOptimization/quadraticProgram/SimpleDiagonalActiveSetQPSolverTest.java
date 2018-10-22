package us.ihmc.convexOptimization.quadraticProgram;

import org.junit.jupiter.api.Test;

public class SimpleDiagonalActiveSetQPSolverTest extends AbstractSimpleActiveSetQPSolverTest
{
   @Override
   public SimpleActiveSetQPSolverInterface createSolverToTest()
   {
      return new SimpleDiagonalActiveSetQPSolver();
   }

   @Test
   public void testSimpleCasesWithBoundsConstraints()
   {
      testSimpleCasesWithBoundsConstraints(1, 3, 2, 3, false);
   }
}

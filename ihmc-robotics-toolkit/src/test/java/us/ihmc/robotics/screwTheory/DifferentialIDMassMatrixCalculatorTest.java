package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

public class DifferentialIDMassMatrixCalculatorTest extends MassMatrixCalculatorTest
{

	@Test
   public void testKineticEnergy()
   {
      setUpRandomChainRobot();
      double expectedKineticEnergy = computeKineticEnergy(joints);

      DifferentialIDMassMatrixCalculator massMatrixCalculator = new DifferentialIDMassMatrixCalculator(worldFrame, elevator);
      massMatrixCalculator.compute();
      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();
      double kineticEnergyFromMassMatrix = computeKineticEnergy(joints, massMatrix);

      assertEquals(expectedKineticEnergy, kineticEnergyFromMassMatrix, 1e-12);
   }
}

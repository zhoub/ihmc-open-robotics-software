package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;

public class JointSpaceConstraintResolver
{
   private final DenseMatrix64F aJointSpaceVdotJointSpace;

   public JointSpaceConstraintResolver()
   {
      int size = Momentum.SIZE;
      this.aJointSpaceVdotJointSpace = new DenseMatrix64F(size, 1);
   }

   public void handleJointSpaceAcceleration(DenseMatrix64F aHat, DenseMatrix64F b, DenseMatrix64F centroidalMomentumMatrix, InverseDynamicsJoint joint,
           DenseMatrix64F jointSpaceAcceleration)
   {
      CommonOps.mult(aHat, jointSpaceAcceleration, aJointSpaceVdotJointSpace);
      CommonOps.subEquals(b, aJointSpaceVdotJointSpace);
   }

   public void solveAndSetJointspaceAccelerations(Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations)
   {
      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         joint.setDesiredAcceleration(jointSpaceAccelerations.get(joint), 0);
      }
   }
}

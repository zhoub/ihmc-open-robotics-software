package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedCenterOfPressureTools
{
   /**
    * Compute center of pressure position given sole positions and vertical ground reaction forces.
    * @param copPosition center of pressure position (output)
    * @param solePosition contact position for each quadrant (input)
    * @param contactPressure vertical ground reaction forces for each quadrant (input)
    */
   public static void computeCenterOfPressure(FixedFramePoint3DBasics copPosition, QuadrantDependentList<FramePoint3D> solePosition,
                                              QuadrantDependentList<MutableDouble> contactPressure)
   {
      // Compute center of pressure given the vertical force at each contact.
      double pressure = 0.0;
      copPosition.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      copPosition.setToZero();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         pressure += contactPressure.get(robotQuadrant).doubleValue();
         solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         copPosition.scaleAdd(contactPressure.get(robotQuadrant).doubleValue(), solePosition.get(robotQuadrant), copPosition);
      }
      if (pressure > 0.0)
         copPosition.scale(1.0 / pressure);
      else
         copPosition.setToNaN();
   }

   /**
    * Compute nominal pressure distribution for a given contact state.
    * @param contactPressure nominal vertical ground reaction forces for each quadrant
    * @param contactState contact state for each quadrant
    */
   public static void computeNominalNormalizedContactPressure(QuadrantDependentList<MutableDouble> contactPressure,
                                                              QuadrantDependentList<ContactState> contactState)
   {
      // Compute vertical force distribution assuming equal loading of hind and front ends.
      int numberOfHindFeetInContact = 0;
      int numberOfFrontFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            if (robotQuadrant.isQuadrantInFront())
            {
               numberOfFrontFeetInContact++;
            }
            else
            {
               numberOfHindFeetInContact++;
            }
            contactPressure.get(robotQuadrant).setValue(1.0);
         }
         else
         {
            contactPressure.get(robotQuadrant).setValue(0.0);
         }
      }

      double numberOfEndsInContact = 0.0;
      if ((numberOfHindFeetInContact > 0) ^ (numberOfFrontFeetInContact > 0))
      {
         numberOfEndsInContact = 1.0;
      }
      if ((numberOfHindFeetInContact > 0) && (numberOfFrontFeetInContact > 0))
      {
         numberOfEndsInContact = 2.0;
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double pressure = contactPressure.get(robotQuadrant).doubleValue();
         pressure /= Math.max(numberOfEndsInContact, 1.0);
         pressure /= Math.max((robotQuadrant.isQuadrantInFront() ? numberOfFrontFeetInContact : numberOfHindFeetInContact), 1.0);
         contactPressure.get(robotQuadrant).setValue(pressure);
      }
   }
}

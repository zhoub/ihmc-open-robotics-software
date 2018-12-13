package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

public class LinearCoMTrajectoryPlannerTools
{
   static final double sufficientlyLarge = 1.0e10;

   static void constructDesiredCoMPosition(FixedFramePoint3DBasics desiredCoMPositionToPack, FramePoint3DReadOnly firstCoefficient,
                                           FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                           FramePoint3DReadOnly fourthCoefficient, ContactState contactState, double timeInPhase, double omega, double gravityZ)
   {
      double firstCoefficientPositionMultiplier = getFirstCoefficientCoMPositionMultiplier(contactState, timeInPhase, omega);
      double secondCoefficientPositionMultiplier = getSecondCoefficientCoMPositionMultiplier(contactState, timeInPhase, omega);
      double gravityPositionEffect = getGravityPositionEffect(contactState, timeInPhase, gravityZ);

      desiredCoMPositionToPack.setToZero();
      desiredCoMPositionToPack.scaleAdd(firstCoefficientPositionMultiplier, firstCoefficient, desiredCoMPositionToPack);
      desiredCoMPositionToPack.scaleAdd(secondCoefficientPositionMultiplier, secondCoefficient, desiredCoMPositionToPack);
      desiredCoMPositionToPack.addZ(gravityPositionEffect);
      if (contactState.isLoadBearing())
      {
         double thirdCoefficientPositionMultiplier = getThirdCoefficientCoMPositionMultiplier(contactState, timeInPhase);
         double fourthCoefficientPositionMultiplier = getFourthCoefficientCoMPositionMultiplier(contactState);

         desiredCoMPositionToPack.scaleAdd(thirdCoefficientPositionMultiplier, thirdCoefficient, desiredCoMPositionToPack);
         desiredCoMPositionToPack.scaleAdd(fourthCoefficientPositionMultiplier, fourthCoefficient, desiredCoMPositionToPack);
      }
   }

   static void constructDesiredCoMVelocity(FixedFrameVector3DBasics desiredCoMVelocityToPack, FramePoint3DReadOnly firstCoefficient,
                                           FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                           FramePoint3DReadOnly fourthCoefficient, ContactState contactState, double timeInPhase, double omega, double gravityZ)
   {
      double firstCoefficientVelocityMultiplier = getFirstCoefficientCoMVelocityMultiplier(contactState, timeInPhase, omega);
      double secondCoefficientVelocityMultiplier = getSecondCoefficientCoMVelocityMultiplier(contactState, timeInPhase, omega);
      double gravityVelocityEffect = getGravityVelocityEffect(contactState, timeInPhase, gravityZ);

      desiredCoMVelocityToPack.setToZero();
      desiredCoMVelocityToPack.scaleAdd(firstCoefficientVelocityMultiplier, firstCoefficient, desiredCoMVelocityToPack);
      desiredCoMVelocityToPack.scaleAdd(secondCoefficientVelocityMultiplier, secondCoefficient, desiredCoMVelocityToPack);
      desiredCoMVelocityToPack.addZ(gravityVelocityEffect);
      if (contactState.isLoadBearing())
      {
         double thirdCoefficientVelocityMultiplier = getThirdCoefficientCoMVelocityMultiplier(contactState);
         double fourthCoefficientVelocityMultiplier = getFourthCoefficientCoMVelocityMultiplier(contactState);

         desiredCoMVelocityToPack.scaleAdd(thirdCoefficientVelocityMultiplier, thirdCoefficient, desiredCoMVelocityToPack);
         desiredCoMVelocityToPack.scaleAdd(fourthCoefficientVelocityMultiplier, fourthCoefficient, desiredCoMVelocityToPack);
      }
   }

   static void constructDesiredCoMAcceleration(FixedFrameVector3DBasics desiredCoMAccelerationToPack, FramePoint3DReadOnly firstCoefficient,
                                               FramePoint3DReadOnly secondCoefficient, FramePoint3DReadOnly thirdCoefficient,
                                               FramePoint3DReadOnly fourthCoefficient, ContactState contactState, double timeInPhase, double omega,
                                               double gravityZ)
   {
      double firstCoefficientAccelerationMultiplier = getFirstCoefficientCoMAccelerationMultiplier(contactState, timeInPhase, omega);
      double secondCoefficientAccelerationMultiplier = getSecondCoefficientCoMAccelerationMultiplier(contactState, timeInPhase, omega);
      double gravityAccelerationEffect = getGravityAccelerationEffect(contactState, gravityZ);

      desiredCoMAccelerationToPack.setToZero();
      desiredCoMAccelerationToPack.scaleAdd(firstCoefficientAccelerationMultiplier, firstCoefficient, desiredCoMAccelerationToPack);
      desiredCoMAccelerationToPack.scaleAdd(secondCoefficientAccelerationMultiplier, secondCoefficient, desiredCoMAccelerationToPack);
      desiredCoMAccelerationToPack.addZ(gravityAccelerationEffect);
      if (contactState.isLoadBearing())
      {
         double thirdCoefficientVelocityMultiplier = getThirdCoefficientCoMAccelerationMultiplier(contactState);
         double fourthCoefficientVelocityMultiplier = getFourthCoefficientCoMAccelerationMultiplier(contactState);

         desiredCoMAccelerationToPack.scaleAdd(thirdCoefficientVelocityMultiplier, thirdCoefficient, desiredCoMAccelerationToPack);
         desiredCoMAccelerationToPack.scaleAdd(fourthCoefficientVelocityMultiplier, fourthCoefficient, desiredCoMAccelerationToPack);
      }
   }

   static double getFirstCoefficientCoMPositionMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState.isLoadBearing())
      {
         return Math.min(sufficientlyLarge, Math.exp(omega * timeInPhase));
      }
      else
      {
         return timeInPhase;
      }
   }

   static double getSecondCoefficientCoMPositionMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState.isLoadBearing())
      {
         return Math.exp(-omega * timeInPhase);
      }
      else
      {
         return 1.0;
      }
   }

   static double getThirdCoefficientCoMPositionMultiplier(ContactState contactState, double timeInPhase)
   {
      if (contactState.isLoadBearing())
      {
         return timeInPhase;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getFourthCoefficientCoMPositionMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 1.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getFirstCoefficientVRPPositionMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight doesn't use the VRP.");
      }
   }

   static double getSecondCoefficientVRPPositionMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight doesn't use the VRP.");
      }
   }

   static double getThirdCoefficientVRPPositionMultiplier(ContactState contactState, double timeInPhase)
   {
      if (contactState.isLoadBearing())
      {
         return Math.min(sufficientlyLarge, timeInPhase);
      }
      else
      {
         throw new IllegalArgumentException("Flight doesn't use the VRP.");
      }
   }

   static double getFourthCoefficientVRPPositionMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 1.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight doesn't use the VRP.");
      }
   }

   static double getFirstCoefficientCoMVelocityMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState.isLoadBearing())
      {
         return Math.min(sufficientlyLarge, omega * Math.exp(omega * timeInPhase));
      }
      else
      {
         return 1.0;
      }
   }

   static double getSecondCoefficientCoMVelocityMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState.isLoadBearing())
      {
         return -omega * getSecondCoefficientCoMPositionMultiplier(contactState, timeInPhase, omega);
      }
      else
      {
         return 0.0;
      }
   }

   static double getThirdCoefficientCoMVelocityMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 1.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getFourthCoefficientCoMVelocityMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getFirstCoefficientCoMAccelerationMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState.isLoadBearing())
      {
         return Math.min(sufficientlyLarge, MathTools.square(omega) * Math.exp(omega * timeInPhase));
      }
      else
      {
         return 1.0;
      }
   }

   static double getSecondCoefficientCoMAccelerationMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState.isLoadBearing())
      {
         return MathTools.square(omega) * getSecondCoefficientCoMPositionMultiplier(contactState, timeInPhase, omega);
      }
      else
      {
         return 0.0;
      }
   }

   static double getThirdCoefficientCoMAccelerationMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getFourthCoefficientCoMAccelerationMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getGravityPositionEffect(ContactState contactState, double timeInPhase, double gravityZ)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         return -0.5 * gravityZ * MathTools.square(timeInPhase);
      }
   }

   static double getGravityVelocityEffect(ContactState contactState, double timeInPhase, double gravityZ)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         return -gravityZ * timeInPhase;
      }
   }

   static double getGravityAccelerationEffect(ContactState contactState, double gravityZ)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         return -gravityZ;
      }
   }
}

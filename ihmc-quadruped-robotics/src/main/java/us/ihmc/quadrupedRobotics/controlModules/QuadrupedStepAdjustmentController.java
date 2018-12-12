package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedStep;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.LinearInvertedPendulumModel;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedStepCrossoverProjection;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.DeadbandTools;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;

public class QuadrupedStepAdjustmentController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrantDependentList<FixedFrameVector3DBasics> instantaneousStepAdjustments = new QuadrantDependentList<>();
   private final QuadrantDependentList<RateLimitedYoFrameVector> limitedInstantaneousStepAdjustments = new QuadrantDependentList<>();
   private final DoubleParameter maxStepAdjustmentRate = new DoubleParameter("maxStepAdjustmentRate", registry, 5.0);

   private final QuadrantDependentList<YoDouble> dcmStepAdjustmentMultipliers = new QuadrantDependentList<>();
   private final YoFrameVector3D dcmError = new YoFrameVector3D("dcmError", worldFrame, registry);
   private final FrameVector3D dcmErrorWithDeadband = new FrameVector3D();
   private final YoBoolean stepHasBeenAdjusted = new YoBoolean("stepHasBeenAdjusted", registry);

   private final DoubleParameter dcmStepAdjustmentGain = new DoubleParameter("dcmStepAdjustmentGain", registry, 1.0);
   private final DoubleParameter minimumFootstepMultiplier = new DoubleParameter("minimumFootstepMultiplier", registry, 0.0);
   private final DoubleParameter dcmErrorThresholdForStepAdjustment = new DoubleParameter("dcmErrorThresholdForStepAdjustment", registry, 0.0);
   private final DoubleParameter dcmErrorDeadbandForStepAdjustment = new DoubleParameter("dcmErrorDeadbandForStepAdjustment", registry, 0.0);
   private final BooleanParameter useStepAdjustment = new BooleanParameter("useStepAdjustment", registry, true);

   private final QuadrupedControllerToolbox controllerToolbox;
   private final QuadrupedStepCrossoverProjection crossoverProjection;
   private final LinearInvertedPendulumModel lipModel;

   private final RecyclingArrayList<QuadrupedStep> adjustedActiveSteps;

   private final YoDouble controllerTime;

   private final FramePoint3D dcmPositionSetpoint = new FramePoint3D();
   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();
   private final FramePoint3D tempPoint = new FramePoint3D();

   public QuadrupedStepAdjustmentController(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.controllerTime = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.lipModel = controllerToolbox.getLinearInvertedPendulumModel();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getShortName();

         YoFrameVector3D instantaneousStepAdjustment = new YoFrameVector3D(prefix + "InstantaneousStepAdjustment", worldFrame, registry);
         RateLimitedYoFrameVector limitedInstantaneousStepAdjustment = new RateLimitedYoFrameVector(prefix + "LimitedInstantaneousStepAdjustment", "", registry,
                                                                                                    maxStepAdjustmentRate,
                                                                                                    controllerToolbox.getRuntimeEnvironment().getControlDT(),
                                                                                                    instantaneousStepAdjustment);

         YoDouble dcmStepAdjustmentMultiplier = new YoDouble(prefix + "DcmStepAdjustmentMultiplier", registry);
         instantaneousStepAdjustment.setToNaN();
         limitedInstantaneousStepAdjustment.setToZero();
         dcmStepAdjustmentMultiplier.setToNaN();

         instantaneousStepAdjustments.put(robotQuadrant, instantaneousStepAdjustment);
         limitedInstantaneousStepAdjustments.put(robotQuadrant, limitedInstantaneousStepAdjustment);
         dcmStepAdjustmentMultipliers.put(robotQuadrant, dcmStepAdjustmentMultiplier);
      }

      adjustedActiveSteps = new RecyclingArrayList<>(10, QuadrupedStep::new);
      adjustedActiveSteps.clear();

      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      crossoverProjection = new QuadrupedStepCrossoverProjection(referenceFrames.getBodyZUpFrame(), referenceFrames.getSoleFrames(), registry);

      parentRegistry.addChild(registry);
   }

   public void completedStep(RobotQuadrant robotQuadrant)
   {
      instantaneousStepAdjustments.get(robotQuadrant).setToNaN();
      limitedInstantaneousStepAdjustments.get(robotQuadrant).setToZero();
      dcmStepAdjustmentMultipliers.get(robotQuadrant).setToNaN();
   }

   public RecyclingArrayList<QuadrupedStep> computeStepAdjustment(ArrayList<YoQuadrupedTimedStep> activeSteps, FramePoint3DReadOnly desiredDCMPosition)
   {
      adjustedActiveSteps.clear();

      // compute step adjustment for ongoing steps (proportional to dcm tracking error)
      controllerToolbox.getDCMPositionEstimate(dcmPositionEstimate);
      dcmPositionEstimate.changeFrame(worldFrame);
      dcmPositionSetpoint.setIncludingFrame(desiredDCMPosition);
      dcmPositionSetpoint.changeFrame(worldFrame);

      dcmError.sub(dcmPositionSetpoint, dcmPositionEstimate);

      boolean stepHasBeenAdjusted = false;

      // adjust nominal step goal positions in foot state machine
      for (int i = 0; i < activeSteps.size(); i++)
      {
         YoQuadrupedTimedStep activeStep = activeSteps.get(i);
         QuadrupedStep adjustedStep = adjustedActiveSteps.add();
         adjustedStep.set(activeStep);

         RobotQuadrant robotQuadrant = activeStep.getRobotQuadrant();

         FixedFrameVector3DBasics instantaneousStepAdjustment = instantaneousStepAdjustments.get(robotQuadrant);
         RateLimitedYoFrameVector limitedInstantaneousStepAdjustment = limitedInstantaneousStepAdjustments.get(robotQuadrant);

         if (instantaneousStepAdjustment.containsNaN())
         {
            limitedInstantaneousStepAdjustment.update();
            limitedInstantaneousStepAdjustment.setToZero();
         }

         YoDouble dcmStepAdjustmentMultiplier = dcmStepAdjustmentMultipliers.get(robotQuadrant);

         if (useStepAdjustment.getValue() && (dcmError.length() > dcmErrorThresholdForStepAdjustment.getValue() || instantaneousStepAdjustment.length() > 0.0))
         {
            dcmErrorWithDeadband.set(dcmError);

            if (DeadbandTools.applyDeadband(dcmErrorWithDeadband, dcmErrorDeadbandForStepAdjustment.getValue()))
            {
               double timeRemainingInStep = Math.max(activeStep.getTimeInterval().getEndTime() - controllerTime.getDoubleValue(), 0.0);
               double recursionMultiplier = Math.exp(timeRemainingInStep * lipModel.getNaturalFrequency());
               double adjustmentMultiplier = minimumFootstepMultiplier.getValue() + (1.0 - minimumFootstepMultiplier.getValue()) * recursionMultiplier;
               dcmStepAdjustmentMultiplier.set(dcmStepAdjustmentGain.getValue() * adjustmentMultiplier);

               instantaneousStepAdjustment.set(dcmError);
               instantaneousStepAdjustment.scale(-dcmStepAdjustmentMultiplier.getDoubleValue());
               instantaneousStepAdjustment.setZ(0);

               stepHasBeenAdjusted = true;
            }
            else
            {
               stepHasBeenAdjusted = false;
            }
         }
         else
         {
            instantaneousStepAdjustment.setToZero();
         }
         limitedInstantaneousStepAdjustment.update();

         activeStep.getGoalPositionProvider(tempPoint);
         tempPoint.changeFrame(worldFrame);
         tempPoint.add(limitedInstantaneousStepAdjustment);
         crossoverProjection.project(tempPoint, robotQuadrant);
         //         groundPlaneEstimator.projectZ(tempPoint);
         adjustedStep.setGoalPosition(tempPoint);
      }

      this.stepHasBeenAdjusted.set(stepHasBeenAdjusted);

      return adjustedActiveSteps;
   }

   public FrameVector3DReadOnly getStepAdjustment(RobotQuadrant robotQuadrant)
   {
      return limitedInstantaneousStepAdjustments.get(robotQuadrant);
   }

   public boolean stepHasBeenAdjusted()
   {
      return stepHasBeenAdjusted.getBooleanValue();
   }
}

package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.RateBasedDesiredHeadingControlModule.DesiredHeadingFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleDesiredHeadingControlModule implements DesiredHeadingControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble desiredHeadingFinal = new YoDouble("desiredHeadingFinal",
         "Yaw of the desired heading frame with respect to the world.", registry);
   private final YoDouble desiredHeading = new YoDouble("desiredHeading", registry);
   private final YoDouble maxHeadingDot = new YoDouble("maxHeadingDot", "In units of rad/sec", registry);

   private final DesiredHeadingFrame desiredHeadingFrame = new DesiredHeadingFrame();
   private final DesiredHeadingFrame predictedHeadingFrame = new DesiredHeadingFrame();

   private final double controlDT;

   public SimpleDesiredHeadingControlModule(double desiredHeadingfinal, double controlDT, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.controlDT = controlDT;

      maxHeadingDot.set(0.1);

      desiredHeadingFinal.set(desiredHeadingfinal);
      desiredHeading.set(desiredHeadingFinal.getDoubleValue()); // The final is the first one according to the initial setup of the robot

      updateDesiredHeadingFrame();
   }

   public void setMaxHeadingDot(double maxHeadingDot)
   {
      this.maxHeadingDot.set(maxHeadingDot);
   }

   public double getMaxHeadingDot()
   {
      return maxHeadingDot.getDoubleValue();
   }

   @Override
   public void updateDesiredHeadingFrame()
   {
      updateDesiredHeading();
      desiredHeadingFrame.setHeadingAngleAndUpdate(desiredHeading.getDoubleValue());
   }

   @Override
   public double getFinalHeadingTargetAngle()
   {
      return desiredHeadingFinal.getDoubleValue();
   }

   @Override
   public FrameVector2D getFinalHeadingTarget()
   {
      FrameVector2D finalHeading = new FrameVector2D(ReferenceFrame.getWorldFrame(), Math.cos(desiredHeadingFinal.getDoubleValue()),
            Math.sin(desiredHeadingFinal.getDoubleValue()));

      return finalHeading;
   }

   @Override
   public ReferenceFrame getDesiredHeadingFrame()
   {
      return desiredHeadingFrame;
   }

   @Override
   public ReferenceFrame getPredictedHeadingFrame(double timeFromNow)
   {
      predictedHeadingFrame.setHeadingAngleAndUpdate(predictDesiredHeading(timeFromNow));
      return predictedHeadingFrame;
   }

   @Override
   public void setFinalHeadingTargetAngle(double finalHeadingTargetAngle)
   {
      desiredHeadingFinal.set(finalHeadingTargetAngle);
   }

   @Override
   public void setFinalHeadingTarget(FrameVector2D finalHeadingTarget)
   {
      finalHeadingTarget.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setFinalHeadingTargetAngle(Math.atan2(finalHeadingTarget.getY(), finalHeadingTarget.getX()));
   }

   @Override
   public double getDesiredHeadingAngle()
   {
      return desiredHeading.getDoubleValue();
   }

   @Override
   public void getDesiredHeading(FrameVector2D desiredHeadingToPack, double timeFromNow)
   {
      double heading = predictDesiredHeading(timeFromNow);
      desiredHeadingToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), Math.cos(heading), Math.sin(heading));
   }

   @Override
   public void resetHeadingAngle(double newHeading)
   {
      desiredHeading.set(newHeading);
      desiredHeadingFinal.set(newHeading);
   }

   private void updateDesiredHeading()
   {
      double error = desiredHeadingFinal.getDoubleValue() - desiredHeading.getDoubleValue();
      double maximumChangePerTick = maxHeadingDot.getDoubleValue() * controlDT;

      double deltaHeading = MathTools.clamp(error, -maximumChangePerTick, maximumChangePerTick);

      desiredHeading.set(desiredHeading.getDoubleValue() + deltaHeading);
   }

   private double predictDesiredHeading(double timeFromNow)
   {
      double error = desiredHeadingFinal.getDoubleValue() - desiredHeading.getDoubleValue();
      double maximumChange = maxHeadingDot.getDoubleValue() * timeFromNow;

      double deltaHeading = MathTools.clamp(error, -maximumChange, maximumChange);

      return desiredHeading.getDoubleValue() + deltaHeading;
   }
}
package us.ihmc.quadrupedCommunication.teleop;

import net.java.games.input.Event;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.stepStream.input.InputValueIntegrator;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Map;

public class QuadrupedStepTeleopMode
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleParameter yawScaleParameter = new DoubleParameter("yawScale", registry, 0.15);
   private final DoubleParameter pitchScaleParameter = new DoubleParameter("pitchScale", registry, 0.15);
   private final DoubleParameter bodyHeightMaxVelocity = new DoubleParameter("bodyHeightMaxVelocity", registry, 0.1);
   private final DoubleParameter xStrideMax = new DoubleParameter("xStrideMax", registry, 0.4);
   private final DoubleParameter yStrideMax = new DoubleParameter("yStrideMax", registry, 0.25);
   private final DoubleParameter yawRateScale = new DoubleParameter("yawRateScale", registry, 0.25);

   // xgait step parameters
   private final DoubleParameter[] xGaitStepDuration = new DoubleParameter[2];
   private final DoubleParameter[] xGaitEndDoubleSupportDuration = new DoubleParameter[2];
   private final DoubleParameter[] xGaitEndPhaseShift = new DoubleParameter[2];

   private final DoubleParameter xGaitBodyOrientationShiftTime = new DoubleParameter("xGaitBodyOrientationShiftTime", registry, 0.1);

   private final QuadrupedTeleopManager stepTeleopManager;
   private InputValueIntegrator bodyHeight;

   public QuadrupedStepTeleopMode(String robotName, Ros2Node ros2Node, double nominalBodyHeight, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                  QuadrupedReferenceFrames referenceFrames, double updateDT, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.stepTeleopManager = new QuadrupedTeleopManager(robotName, ros2Node, xGaitSettings, nominalBodyHeight, referenceFrames, graphicsListRegistry, registry);
      this.bodyHeight = new InputValueIntegrator(updateDT, nominalBodyHeight);

      xGaitStepDuration[0] = new DoubleParameter("xGaitStepDurationMode0", registry, 0.5);
      xGaitStepDuration[1] = new DoubleParameter("xGaitStepDurationMode1", registry, 0.33);
      xGaitEndDoubleSupportDuration[0] = new DoubleParameter("xGaitEndDoubleSupportDurationMode0", registry, 1.0);
      xGaitEndDoubleSupportDuration[1] = new DoubleParameter("xGaitEndDoubleSupportDurationMode1", registry, 0.05);
      xGaitEndPhaseShift[0] = new DoubleParameter("xGaitEndPhaseShiftMode0", registry, 90);
      xGaitEndPhaseShift[1] = new DoubleParameter("xGaitEndPhaseShiftMode1", registry, 180);

      parentRegistry.addChild(registry);
   }

   public void update(Map<XBoxOneMapping, Double> channels)
   {
      if (channels != null)
      {
         double bodyRoll = 0.0;
         double bodyPitch = channels.get(XBoxOneMapping.RIGHT_STICK_Y) * pitchScaleParameter.getValue();
         double bodyYaw = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawScaleParameter.getValue();

         double bodyHeightVelocity = 0.0;
         if (channels.get(XBoxOneMapping.DPAD) == 0.25)
         {
            bodyHeightVelocity += bodyHeightMaxVelocity.getValue();
         }
         if (channels.get(XBoxOneMapping.DPAD) == 0.75)
         {
            bodyHeightVelocity -= bodyHeightMaxVelocity.getValue();
         }
         stepTeleopManager.setDesiredBodyHeight(bodyHeight.update(bodyHeightVelocity));

         stepTeleopManager.setDesiredBodyPose(0.0, 0.0,  bodyYaw, bodyPitch, bodyRoll, xGaitBodyOrientationShiftTime.getValue());


         YoQuadrupedXGaitSettings xGaitSettings = stepTeleopManager.getXGaitSettings();
         double xVelocityMax = 0.5 * xStrideMax.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
         double yVelocityMax = 0.5 * yStrideMax.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
         double yawRateMax = yawRateScale.getValue() / (xGaitSettings.getStepDuration() + xGaitSettings.getEndDoubleSupportDuration());
         double xVelocity = channels.get(XBoxOneMapping.LEFT_STICK_Y) * xVelocityMax;
         double yVelocity = channels.get(XBoxOneMapping.LEFT_STICK_X) * yVelocityMax;
         double yawRate = channels.get(XBoxOneMapping.RIGHT_STICK_X) * yawRateMax;
         stepTeleopManager.setDesiredVelocity(xVelocity, yVelocity, yawRate);
      }
      stepTeleopManager.update();
   }

   public void onInputEvent(Map<XBoxOneMapping, Double> channels, Event event)
   {
      if (event.getValue() < 0.5)
         return;

      if (XBoxOneMapping.getMapping(event) == XBoxOneMapping.A)
      {
         stepTeleopManager.requestWalkingState();
         if (stepTeleopManager.isWalking())
            stepTeleopManager.requestStanding();
      }

      if (XBoxOneMapping.getMapping(event) == XBoxOneMapping.X)
      {
         stepTeleopManager.requestXGait();
      }

      YoQuadrupedXGaitSettings xGaitSettings = stepTeleopManager.getXGaitSettings();
      switch (XBoxOneMapping.getMapping(event))
      {
      case RIGHT_BUMPER:
         xGaitSettings.setStepDuration(xGaitStepDuration[0].getValue());
         xGaitSettings.setEndDoubleSupportDuration(xGaitEndDoubleSupportDuration[0].getValue());
         xGaitSettings.setEndPhaseShift(xGaitEndPhaseShift[0].getValue());
         break;
      case LEFT_BUMPER:
         xGaitSettings.setStepDuration(xGaitStepDuration[1].getValue());
         xGaitSettings.setEndDoubleSupportDuration(xGaitEndDoubleSupportDuration[1].getValue());
         xGaitSettings.setEndPhaseShift(xGaitEndPhaseShift[1].getValue());
         break;
      }
   }
}

package us.ihmc.quadrupedRobotics;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedForceTestYoVariables extends QuadrupedTestYoVariables
{
   private final YoEnum<HighLevelControllerName> userTrigger;
   private final YoEnum<QuadrupedSteppingRequestedEvent> stepTrigger;
   private final YoEnum<HighLevelControllerName> controllerState;
   private final YoEnum<QuadrupedSteppingStateEnum> steppingState;

   private final YoDouble stanceHeight;
   private final YoDouble groundPlanePointZ;
   
   // Step
   private final YoEnum<RobotQuadrant> timedStepQuadrant;
   private final YoDouble timedStepDuration;
   private final YoDouble timedStepGroundClearance;
   private final YoDouble timedStepGoalPositionX;
   private final YoDouble timedStepGoalPositionY;
   private final YoDouble timedStepGoalPositionZ;

   // CoM
   private final YoDouble comPositionEstimateX;
   private final YoDouble comPositionEstimateY;
   private final YoDouble currentHeightInWorld;

   private final YoDouble bodyCurrentOrientationQx;
   private final YoDouble bodyCurrentOrientationQy;
   private final YoDouble bodyCurrentOrientationQz;
   private final YoDouble bodyCurrentOrientationQs;

   private final YoDouble comPositionSetpointX;
   private final YoDouble comPositionSetpointY;
   private final YoDouble desiredHeightInWorld;

   private final YoDouble timeToPrepareForStanding;
   private final YoDouble minimumTimeDoneWithStandPrep;

   private final YoDouble toWalkingTransitionDuration;
   private final YoDouble timeToMoveSittingDown;

   private final Quaternion bodyOrientation = new Quaternion();

   @SuppressWarnings("unchecked")
   public QuadrupedForceTestYoVariables(SimulationConstructionSet scs)
   {
      super(scs);
      
      userTrigger = (YoEnum<HighLevelControllerName>) scs.getVariable("requestedControllerState");
      stepTrigger = (YoEnum<QuadrupedSteppingRequestedEvent>) scs.getVariable("stepTrigger");
      controllerState = (YoEnum<HighLevelControllerName>) scs.getVariable("controllerCurrentState");
      steppingState = (YoEnum<QuadrupedSteppingStateEnum>) scs.getVariable("steppingCurrentState");

      stanceHeight = (YoDouble) scs.getVariable("stanceHeight");
      groundPlanePointZ = (YoDouble) scs.getVariable("groundPlanePointZ");
      
      comPositionEstimateX = (YoDouble) scs.getVariable("comPositionEstimateX");
      comPositionEstimateY = (YoDouble) scs.getVariable("comPositionEstimateY");
      currentHeightInWorld = (YoDouble) scs.getVariable("currentHeightInWorld");
      
      timedStepQuadrant = (YoEnum<RobotQuadrant>) scs.getVariable("timedStepQuadrant");
      timedStepDuration = (YoDouble) scs.getVariable("timedStepDuration");
      timedStepGroundClearance = (YoDouble) scs.getVariable("timedStepGroundClearance");
      timedStepGoalPositionX = (YoDouble) scs.getVariable("timedStepGoalPositionX");
      timedStepGoalPositionY = (YoDouble) scs.getVariable("timedStepGoalPositionY");
      timedStepGoalPositionZ = (YoDouble) scs.getVariable("timedStepGoalPositionZ");

      bodyCurrentOrientationQx = (YoDouble) scs.getVariable("bodyCurrentOrientationQx");
      bodyCurrentOrientationQy = (YoDouble) scs.getVariable("bodyCurrentOrientationQy");
      bodyCurrentOrientationQz = (YoDouble) scs.getVariable("bodyCurrentOrientationQz");
      bodyCurrentOrientationQs = (YoDouble) scs.getVariable("bodyCurrentOrientationQs");

      comPositionSetpointX = (YoDouble) scs.getVariable("comPositionSetpointX");
      comPositionSetpointY = (YoDouble) scs.getVariable("comPositionSetpointY");
      desiredHeightInWorld = (YoDouble) scs.getVariable("desiredHeightInWorld");

      timeToPrepareForStanding = (YoDouble) scs.getVariable("timeToPrepareForStanding");
      minimumTimeDoneWithStandPrep = (YoDouble) scs.getVariable("minimumTimeDoneWithStandPrep");
      toWalkingTransitionDuration = (YoDouble) scs.getVariable("toWalkingTransitionDuration");
      timeToMoveSittingDown = (YoDouble) scs.getVariable("timeToMoveSittingDown");
   }

   public YoEnum<HighLevelControllerName> getUserTrigger()
   {
      return userTrigger;
   }

   public YoEnum<QuadrupedSteppingRequestedEvent> getStepTrigger()
   {
      return stepTrigger;
   }

   public YoEnum<HighLevelControllerName> getControllerState()
   {
      return controllerState;
   }

   public YoEnum<QuadrupedSteppingStateEnum> getSteppingState()
   {
      return steppingState;
   }

   public YoDouble getStanceHeight()
   {
      return stanceHeight;
   }

   public YoDouble getGroundPlanePointZ()
   {
      return groundPlanePointZ;
   }

   public YoDouble getComPositionEstimateX()
   {
      return comPositionEstimateX;
   }

   public YoDouble getComPositionEstimateY()
   {
      return comPositionEstimateY;
   }

   public YoDouble getCurrentHeightInWorld()
   {
      return currentHeightInWorld;
   }
   
   public YoEnum<RobotQuadrant> getTimedStepQuadrant()
   {
      return timedStepQuadrant;
   }

   public YoDouble getTimedStepDuration()
   {
      return timedStepDuration;
   }

   public YoDouble getTimedStepGroundClearance()
   {
      return timedStepGroundClearance;
   }

   public YoDouble getTimedStepGoalPositionX()
   {
      return timedStepGoalPositionX;
   }

   public YoDouble getTimedStepGoalPositionY()
   {
      return timedStepGoalPositionY;
   }

   public YoDouble getTimedStepGoalPositionZ()
   {
      return timedStepGoalPositionZ;
   }

   public double getBodyEstimateYaw()
   {
      bodyOrientation.set(bodyCurrentOrientationQx.getDoubleValue(), bodyCurrentOrientationQy.getDoubleValue(), bodyCurrentOrientationQz.getDoubleValue(),
                          bodyCurrentOrientationQs.getDoubleValue());
      return bodyOrientation.getYaw();
   }

   public double getBodyEstimatePitch()
   {
       bodyOrientation.set(bodyCurrentOrientationQx.getDoubleValue(), bodyCurrentOrientationQy.getDoubleValue(), bodyCurrentOrientationQz.getDoubleValue(),
                                 bodyCurrentOrientationQs.getDoubleValue());
      return bodyOrientation.getPitch();
   }

   public double getBodyEstimateRoll()
   {
      bodyOrientation.set(bodyCurrentOrientationQx.getDoubleValue(), bodyCurrentOrientationQy.getDoubleValue(), bodyCurrentOrientationQz.getDoubleValue(),
                          bodyCurrentOrientationQs.getDoubleValue());
      return bodyOrientation.getRoll();
   }

   public YoDouble getComPositionSetpointX()
   {
      return comPositionSetpointX;
   }

   public YoDouble getComPositionSetpointY()
   {
      return comPositionSetpointY;
   }

   public YoDouble getHeightInWorldSetpoint()
   {
      return desiredHeightInWorld;
   }

   public YoDouble getTimeToPrepareForStanding()
   {
      return timeToPrepareForStanding;
   }

   public YoDouble getMinimumTimeDoneWithStandPrep()
   {
      return minimumTimeDoneWithStandPrep;
   }

   public double getTimeInStandPrep()
   {
      return timeToPrepareForStanding.getDoubleValue() + minimumTimeDoneWithStandPrep.getDoubleValue();
   }

   public double getToWalkingTransitionDuration()
   {
      return toWalkingTransitionDuration.getDoubleValue();
   }

   public double getTimeToMoveSittingDown()
   {
      return timeToMoveSittingDown.getDoubleValue();
   }
}

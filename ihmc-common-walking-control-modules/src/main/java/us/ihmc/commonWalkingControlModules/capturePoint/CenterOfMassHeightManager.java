package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * this class manages the center of mass height or the pelvis height using the PelvisHeightTrajectoryCommand and PelvisTrajectoryCommand respectively
 * PelvisTrajectoryCommand is considered a special user command which gets forwarded to PelvisHeightControlState,
 * In this user mode, the controller won't change the height of the pelvis to ensure the legs don't reach singularities while swinging. You must
 * take the robot's configuration into account while using this command. The PelvisTrajectoryCommand also allows the user to enable User Pelvis Control During Walking.
 * If this is turned off, the controller will switch back to CenterOfMassHeightControlState during walking
 * Only the Z component of the PelvisTrajectoryCommand is used to control the pelvis height.
 *
 * The PelvisHeightTrajectoryCommand uses a pdController to compute the Linear Momentum Z and sends a momentum command to the controller core
 * If you want to the controller to manage the pelvis height while walking use the PelvisHeightTrajectoryCommand.
 */
public class CenterOfMassHeightManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   /** User Controlled Pelvis Height Mode, tries to achieve a desired pelvis height regardless of the robot configuration**/
   private final PelvisHeightControlState pelvisHeightControlState;

   /** if the manager is in user mode before walking then stay in it while walking (PelvisHeightControlState) **/
   private final YoBoolean enableUserPelvisControlDuringWalking = new YoBoolean("centerOfMassHeightManagerEnableUserPelvisControlDuringWalking", registry);

   private final FramePose3D tempPose = new FramePose3D();
   private final FramePoint3D tempPosition = new FramePoint3D();

   public CenterOfMassHeightManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      pelvisHeightControlState = new PelvisHeightControlState(controllerToolbox, walkingControllerParameters, registry);

      enableUserPelvisControlDuringWalking.set(false);
   }

   public void initialize()
   {
      goHome(0.5);
   }

   /**
    * set the weights for user mode, CenterOfMassHeightControlState does not use this weight
    * @param weight
    */
   public void setPelvisTaskspaceWeights(Vector3DReadOnly weight)
   {
      pelvisHeightControlState.setWeights(weight);
   }

   public void compute()
   {
      pelvisHeightControlState.doAction();
   }

   /**
    * sets the height manager up for walking
    * If we're in user mode and not allowed to stay that way while walking then switch out of user mode
    */
   // TODO: move functionality
   public void prepareForLocomotion()
   {
      if (enableUserPelvisControlDuringWalking.getBooleanValue())
         return;

      pelvisHeightControlState.initializeDesiredHeightToCurrent();
   }

   public void initializeDesiredHeightToCurrent()
   {
      pelvisHeightControlState.initializeDesiredHeightToCurrent();
   }

   /**
    * checks that the command is valid and switches to user mode
    * The controller will try to achieve the pelvis height regardless of the robot configuration
    * @param command - only the linear z portion of this command is used
    */
   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
      pelvisHeightControlState.getCurrentDesiredHeightOfDefaultControlFrame(tempPosition);

      tempPose.setToZero(tempPosition.getReferenceFrame());
      tempPose.setPosition(tempPosition);

      pelvisHeightControlState.handlePelvisTrajectoryCommand(command, tempPose);
   }

   /**
    * switches to center of mass height controller, this is the standard height manager
    * the height in this command will be adjusted based on the legs
    * @param command
    */
   public void handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      enableUserPelvisControlDuringWalking.set(command.isEnableUserPelvisControlDuringWalking());
      pelvisHeightControlState.getCurrentDesiredHeightOfDefaultControlFrame(tempPosition);

      tempPose.setToZero(tempPosition.getReferenceFrame());
      tempPose.setPosition(tempPosition);

      pelvisHeightControlState.handlePelvisHeightTrajectoryCommand(command, tempPose);
   }

   /**
    * set the desired height to walkingControllerParameters.nominalHeightAboveAnkle()
    * @param trajectoryTime
    */
   public void goHome(double trajectoryTime)
   {
      pelvisHeightControlState.goHome(trajectoryTime);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      pelvisHeightControlState.handleStopAllTrajectoryCommand(command);
   }

   // TODO: move functionality
   public void setSupportLeg(RobotSide supportLeg)
   {
//      centerOfMassHeightControlState.setSupportLeg(supportLeg);
   }

   // TODO: move functionality
   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();
      Footstep stanceFootstep = transferToAndNextFootstepsData.getTransferFromFootstep();

      if (nextFootstep == null)
      {
         return;
      }

      FramePoint3D nextFootPosition = new FramePoint3D();
      FramePoint3D stanceFootPosition = new FramePoint3D();
      nextFootstep.getPosition(nextFootPosition);
      stanceFootstep.getPosition(stanceFootPosition);

//      centerOfMassHeightControlState.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }

   /**
    * The Desired acceleration of the COM. User mode returns 0, while the center of mass height manager returns the action from the internal pd controller over the height
    * @return
    */
   public double computeDesiredCoMHeightAcceleration(FrameVector2D desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
         FeetManager feetManager)
   {
      return pelvisHeightControlState.computeDesiredCoMHeightAcceleration(desiredICPVelocity, isInDoubleSupport, omega0, isRecoveringFromPush, feetManager);
   }

   // TODO: remove
   public boolean hasBeenInitializedWithNextStep()
   {
//      return centerOfMassHeightControlState.hasBeenInitializedWithNextStep();
      return true;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return pelvisHeightControlState.getFeedbackControlCommand();
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return pelvisHeightControlState.getFeedbackControlCommand();
   }

   public boolean getControlHeightWithMomentum()
   {
      return false;
   }

   public void setComHeightGains(PIDGainsReadOnly userModeComHeightGains)
   {
      pelvisHeightControlState.setGains(userModeComHeightGains);
   }
}

package us.ihmc.avatar.networkProcessor.walkingPreview;

import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Stream;

import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;

public class WalkingPreviewResetTask implements WalkingPreviewTask
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<YoPlaneContactState> footContactStates;
   private final SideDependentList<WalkingPreviewContactPointHolder> contactStateHolders = new SideDependentList<>();
   private final InverseDynamicsCommandList commandList = new InverseDynamicsCommandList();

   private final FullHumanoidRobotModel fullRobotModel;
   private final CommandInputManager walkingInputManager;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final AtomicReference<RobotMotionStatus> latestMotionStatus = new AtomicReference<>(null);
   private RobotMotionStatusChangedListener robotMotionStatusChangedListener = (newStatus, time) -> latestMotionStatus.set(newStatus);

   public WalkingPreviewResetTask(FullHumanoidRobotModel fullRobotModel, SideDependentList<YoPlaneContactState> footContactStates,
                                  CommandInputManager walkingInputManager, HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.fullRobotModel = fullRobotModel;
      this.footContactStates = footContactStates;
      this.walkingInputManager = walkingInputManager;
      this.controllerToolbox = controllerToolbox;
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (RobotSide robotSide : RobotSide.values)
         contactStateHolders.put(robotSide, WalkingPreviewContactPointHolder.holdAtCurrent(footContactStates.get(robotSide)));

      // Get the controller to be initialized to hold the initial robot configuration.

      FramePose3D initialPelvisPose = new FramePose3D(fullRobotModel.getRootJoint().getFrameAfterJoint());
      initialPelvisPose.changeFrame(worldFrame);
      PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(0.01, initialPelvisPose.getPosition(),
                                                                                                           initialPelvisPose.getOrientation());
      walkingInputManager.submitMessage(pelvisTrajectoryMessage);

      RigidBodyBasics chest = fullRobotModel.getChest();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         double[] desiredJointPositions = Stream.of(MultiBodySystemTools.createOneDoFJointPath(chest, hand)).mapToDouble(OneDoFJointBasics::getQ).toArray();
         walkingInputManager.submitMessage(HumanoidMessageTools.createArmTrajectoryMessage(robotSide, 0.01, desiredJointPositions));
      }

      RigidBodyBasics head = fullRobotModel.getHead();
      double[] desiredJointPositions = Stream.of(MultiBodySystemTools.createOneDoFJointPath(chest, head)).mapToDouble(OneDoFJointBasics::getQ).toArray();
      walkingInputManager.submitMessage(HumanoidMessageTools.createNeckTrajectoryMessage(0.01, desiredJointPositions));

      FrameQuaternion initialChestOrientation = new FrameQuaternion(chest.getBodyFixedFrame());
      initialChestOrientation.changeFrame(worldFrame);
      walkingInputManager.submitMessage(HumanoidMessageTools.createChestTrajectoryMessage(0.01, initialChestOrientation, worldFrame));

      controllerToolbox.attachRobotMotionStatusChangedListener(robotMotionStatusChangedListener);
   }

   @Override
   public void doAction()
   {
      commandList.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         contactStateHolders.get(robotSide).doControl();
         commandList.addCommand(contactStateHolders.get(robotSide).getOutput());
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      destroyListeners();
   }

   @Override
   public boolean isDone()
   {
      return true;
   }

   @Override
   public InverseDynamicsCommand<?> getOutput()
   {
      return commandList;
   }

   @Override
   protected void finalize() throws Throwable
   {
      super.finalize();
      destroyListeners(); // In case the doTransitionOutOfAction() was not called somehow.
   }

   private void destroyListeners()
   {
      if (robotMotionStatusChangedListener != null)
      {
         controllerToolbox.detachRobotMotionStatusChangedListener(robotMotionStatusChangedListener);
         robotMotionStatusChangedListener = null;
      }
   }
}

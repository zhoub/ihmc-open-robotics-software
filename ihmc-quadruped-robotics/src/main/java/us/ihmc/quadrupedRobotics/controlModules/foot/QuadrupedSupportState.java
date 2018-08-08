package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedFootControlModuleParameters;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedSupportState extends QuadrupedFootState
{
   private final String name = getClass().getSimpleName();
   private YoVariableRegistry registry;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RobotQuadrant robotQuadrant;
   private final YoPlaneContactState contactState;
   private final QuadrupedFootControlModuleParameters parameters;

   private final SpatialVelocityCommand spatialVelocityCommand = new SpatialVelocityCommand();
   private final FrameVector3D footNormalContactVector = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
   private final SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
   
   private final SelectionMatrix6D spatialSelectionMatrix = new SelectionMatrix6D();
   private final YoBoolean controlAngularX;
   private final YoBoolean controlAngularY;
   private final YoBoolean controlAngularZ;

   private final ReferenceFrame frameToControl;
   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D(worldFrame);


   public QuadrupedSupportState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      String prefix = robotQuadrant.getCamelCaseName();
      registry = new YoVariableRegistry(prefix + name);
      this.robotQuadrant = robotQuadrant;
      this.contactState = controllerToolbox.getFootContactState(robotQuadrant);
      this.parameters = controllerToolbox.getFootControlModuleParameters();

      spatialVelocityCommand.set(controllerToolbox.getFullRobotModel().getElevator(), controllerToolbox.getFullRobotModel().getFoot(robotQuadrant));
      spatialVelocityCommand.setSelectionMatrixForLinearControl();
      spatialVelocityCommand.setWeight(10.0);
      
      controlAngularX = new YoBoolean(prefix + "ControlAngularX", registry);
      controlAngularY = new YoBoolean(prefix + "ControlAngularY", registry);
      controlAngularZ = new YoBoolean(prefix + "ControlAngularZ", registry);
      
      controlAngularX.set(true);
      controlAngularY.set(true);
      controlAngularZ.set(true);
      
      frameToControl = grossHardcodedMethodToGetFrameToControlForRoboMantis(robotQuadrant, controllerToolbox);
      
      desiredOrientation.setToZero(frameToControl);
      desiredOrientation.changeFrame(worldFrame);
      
      parentRegistry.addChild(registry);
   }
   
   private ReferenceFrame grossHardcodedMethodToGetFrameToControlForRoboMantis(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox)
   {
      RigidBody endEffectorToControl = controllerToolbox.getFullRobotModel().getFoot(robotQuadrant);
      MovingReferenceFrame bodyFixedFrame = endEffectorToControl.getBodyFixedFrame();
      
      Vector3D distanceFromBodyFixedFrameToWheelCenter = new Vector3D();
      FramePoint3D distanceFromBodyToWheelCenter = new FramePoint3D(controllerToolbox.getFullRobotModel().getSoleFrame(robotQuadrant));
      distanceFromBodyToWheelCenter.changeFrame(bodyFixedFrame);
      distanceFromBodyFixedFrameToWheelCenter.set(distanceFromBodyToWheelCenter);

      int sign = 1;
      if (robotQuadrant == RobotQuadrant.FRONT_LEFT || robotQuadrant == RobotQuadrant.HIND_RIGHT)
      {
         sign = -1;
      }

      RigidBodyTransform transformToParent = new RigidBodyTransform();
      Pose3D controlPose = new Pose3D();
      controlPose.setOrientation(0.0, 0.0, sign * 0.501, 0.865);
      controlPose.setPosition(distanceFromBodyFixedFrameToWheelCenter);
      controlPose.get(transformToParent);
      String frameName = robotQuadrant.getCamelCaseName() + "ControlledLegFrame";
      ReferenceFrame frameToControl = ReferenceFrame.constructFrameWithUnchangingTransformToParent(frameName, bodyFixedFrame, transformToParent);
      FramePose3D controlFrameFixedInEndEffector = new FramePose3D(frameToControl);
      controlFrameFixedInEndEffector.changeFrame(endEffectorToControl.getBodyFixedFrame());

      feedbackControlCommand.set(controllerToolbox.getFullRobotModel().getElevator(), endEffectorToControl);
      feedbackControlCommand.setControlFrameFixedInEndEffector(controlFrameFixedInEndEffector);

      return frameToControl;
   }

   @Override
   public void onEntry()
   {
      contactState.setFullyConstrained();
      contactState.setContactNormalVector(footNormalContactVector);

      if (waypointCallback != null)
         waypointCallback.isDoneMoving(robotQuadrant, true);
   }

   @Override
   public void doAction(double timeInState)
   {
      spatialVelocityCommand.setSpatialVelocityToZero(frameToControl);
      contactState.setFullyConstrained();
      
      feedbackControlCommand.setAngularWeightsForSolver(parameters.getSoleOrientationWeights());
      feedbackControlCommand.setOrientationGains(parameters.getSoleOrientationGains());
      feedbackControlCommand.setSelectionMatrix(computeSpatialSelectionMatrix());
      feedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity);
   }
   
   private SelectionMatrix6D computeSpatialSelectionMatrix()
   {
      spatialSelectionMatrix.clearSelection();
      spatialSelectionMatrix.selectAngularX(controlAngularX.getBooleanValue());
      spatialSelectionMatrix.selectAngularY(controlAngularY.getBooleanValue());
      spatialSelectionMatrix.selectAngularZ(controlAngularZ.getBooleanValue());

      return spatialSelectionMatrix;
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return null;
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return null;//feedbackControlCommand;
   }

   @Override
   public InverseKinematicsCommand<?> getInverseKinematicsCommand()
   {
      //      return null;
      return spatialVelocityCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }
}

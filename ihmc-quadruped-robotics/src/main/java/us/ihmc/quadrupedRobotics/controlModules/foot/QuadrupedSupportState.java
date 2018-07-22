package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
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
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.SolverWeightLevels;
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

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();
   private final SelectionMatrix6D footAccelerationSelectionMatrix = new SelectionMatrix6D();
   private final ParameterVector3D soleOrientationWeightsID;
   
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
      
      soleOrientationWeightsID = new ParameterVector3D("soleSupportOrientationWeightID_", new Vector3D(10.0, 10.0, 10.0), registry);
      
      controlAngularX = new YoBoolean(prefix + "ControlAngularX", registry);
      controlAngularY = new YoBoolean(prefix + "ControlAngularY", registry);
      controlAngularZ = new YoBoolean(prefix + "ControlAngularZ", registry);
      
      controlAngularX.set(true);
      controlAngularY.set(true);
      controlAngularZ.set(false);
      
      frameToControl = grossHardcodedMethodToGetFrameToControlForRoboMantis(robotQuadrant, controllerToolbox);
      
      RigidBody foot = controllerToolbox.getFullRobotModel().getFoot(robotQuadrant);
      
      spatialAccelerationCommand.setWeight(SolverWeightLevels.VERY_HIGH);
      spatialAccelerationCommand.set(controllerToolbox.getFullRobotModel().getElevator(), foot);
      spatialAccelerationCommand.setPrimaryBase(controllerToolbox.getFullRobotModel().getBody());
      
      footAccelerationSelectionMatrix.setToLinearSelectionOnly();
      footAccelerationSelectionMatrix.setSelectionFrame(worldFrame);
      spatialAccelerationCommand.setSelectionMatrix(footAccelerationSelectionMatrix);
      
      desiredOrientation.setToZero(frameToControl);
      desiredOrientation.changeFrame(worldFrame);
      
      parentRegistry.addChild(registry);
   }
   
   private ReferenceFrame grossHardcodedMethodToGetFrameToControlForRoboMantis(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox)
   {
      OneDoFJoint joint = controllerToolbox.getFullRobotModel().getLegJoint(robotQuadrant, LegJointName.ANKLE_ROLL);
      RigidBody endEffectorToControl = joint.getSuccessor();

      String frameName = robotQuadrant.getCamelCaseName() + "ControlledLegFrame";
      Vector3D distanceFromBodyFixedFrameToWheelCenter = new Vector3D();

      MovingReferenceFrame bodyFixedFrame = endEffectorToControl.getBodyFixedFrame();
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
      ReferenceFrame frameToControl = ReferenceFrame.constructFrameWithUnchangingTransformToParent(frameName, bodyFixedFrame, transformToParent);
      FramePose3D controlFrameFixedInEndEffector = new FramePose3D(frameToControl);
      controlFrameFixedInEndEffector.changeFrame(endEffectorToControl.getBodyFixedFrame());

      feedbackControlCommand.set(controllerToolbox.getFullRobotModel().getElevator(), endEffectorToControl);
      feedbackControlCommand.setPrimaryBase(controllerToolbox.getFullRobotModel().getBody());
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
      
      spatialAccelerationCommand.setSpatialAccelerationToZero(frameToControl);
   }

   @Override
   public void doAction(double timeInState)
   {
      spatialVelocityCommand.setSpatialVelocityToZero(frameToControl);
      contactState.setFullyConstrained();
      
      feedbackControlCommand.setAngularWeightsForSolver(soleOrientationWeightsID);
      feedbackControlCommand.setOrientationGains(parameters.getSupportOrientationGains());
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
      return spatialAccelerationCommand;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }

   @Override
   public InverseKinematicsCommand<?> getInverseKinematicsCommand()
   {
      return spatialVelocityCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }
}

package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedFootControlModuleParameters;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.math.trajectories.HermiteCurveBasedOrientationTrajectoryGenerator;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedSpatialSwingState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   
   // SoleWaypoint variables
   private final TwoWaypointSwingGenerator positionTrajectory;
   private final HermiteCurveBasedOrientationTrajectoryGenerator orientationTrajectory;

   private final QuadrupedSupportState supportState;
   
   // Feedback controller
   private final ReferenceFrame frameToControl;

   private final YoQuadrupedTimedStep currentStepCommand;

   private final QuadrupedFootControlModuleParameters parameters;

   private final VirtualForceCommand virtualForceCommand = new VirtualForceCommand();

   private final SelectionMatrix6D spatialSelectionMatrix = new SelectionMatrix6D();
   private final YoBoolean controlLinearX;
   private final YoBoolean controlLinearY;
   private final YoBoolean controlLinearZ;
   private final YoBoolean controlAngularX;
   private final YoBoolean controlAngularY;
   private final YoBoolean controlAngularZ;
   
   private final YoBoolean notInitialized;

   private final SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();

   private final QuadrupedControllerToolbox controllerToolbox;
   private final RobotQuadrant robotQuadrant;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialLinearVelocity = new FrameVector3D(worldFrame);

   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalLinearVelocity = new FrameVector3D(worldFrame);

   private final FrameQuaternion initialOrientation = new FrameQuaternion();
   private final FrameVector3D initialAngularVelocity = new FrameVector3D();

   private final FrameQuaternion finalOrientation = new FrameQuaternion();
   private final FrameVector3D finalAngularVelocity = new FrameVector3D();

   private final FramePoint3D desiredFootPosition = new FramePoint3D();
   private final FrameVector3D desiredFootLinearVelocity = new FrameVector3D();
   private final FrameVector3D desiredFootLinearAcceleration = new FrameVector3D();

   private final FrameQuaternion desiredFootOrientation = new FrameQuaternion();
   private final FrameVector3D desiredFootAngularVelocity = new FrameVector3D();
   private final FrameVector3D desiredFootAngularAcceleration = new FrameVector3D();
   
   private final YoQuadrupedTimedStep currentStepCommandSource;


   public QuadrupedSpatialSwingState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoQuadrupedTimedStep currentStepCommand,
                                     YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.currentStepCommand = new YoQuadrupedTimedStep("currentStepCommandCopy", registry);
      String prefix = robotQuadrant.getCamelCaseName();
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;
      
      supportState = new QuadrupedSupportState(robotQuadrant, controllerToolbox, registry);

      this.parameters = controllerToolbox.getFootControlModuleParameters();

      positionTrajectory = new TwoWaypointSwingGenerator(prefix, 0.1, 0.4, registry, yoGraphicsListRegistry);
      positionTrajectory.setTrajectoryType(TrajectoryType.DEFAULT);
      positionTrajectory.showVisualization();

      orientationTrajectory = new HermiteCurveBasedOrientationTrajectoryGenerator(prefix + "OrientationTrajectory", worldFrame, registry);

      controlLinearX = new YoBoolean(prefix + "ControlLinearX", registry);
      controlLinearY = new YoBoolean(prefix + "ControlLinearY", registry);
      controlLinearZ = new YoBoolean(prefix + "ControlLinearZ", registry);
      controlAngularX = new YoBoolean(prefix + "ControlAngularX", registry);
      controlAngularY = new YoBoolean(prefix + "ControlAngularY", registry);
      controlAngularZ = new YoBoolean(prefix + "ControlAngularZ", registry);
      
      notInitialized = new YoBoolean("notInitialized", registry);
      
      controlLinearX.set(true);
      controlLinearY.set(true);
      controlLinearZ.set(true);
      controlAngularX.set(true);
      controlAngularY.set(true);
      controlAngularZ.set(true);
      
      currentStepCommandSource = currentStepCommand;

      this.frameToControl = grossHardcodedMethodToGetFrameToControlForRoboMantis(robotQuadrant, controllerToolbox);
      
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

   public void initialize()
   {
     
   }

   @Override
   public void onEntry()
   {
      currentStepCommand.set(currentStepCommandSource);
      supportState.onEntry();
      notInitialized.set(true);
   }

   public void updateTrajectory()
   {
      currentStepCommand.getGoalPosition(finalPosition);
      TimeInterval timeInterval = currentStepCommand.getTimeInterval();

      initialPosition.setToZero(frameToControl);
      initialPosition.changeFrame(worldFrame);

      positionTrajectory.setInitialConditions(initialPosition, initialLinearVelocity);
      positionTrajectory.setFinalConditions(finalPosition, finalLinearVelocity);
      positionTrajectory.setSwingHeight(currentStepCommand.getGroundClearance());
      positionTrajectory.setStepTime(timeInterval.getDuration() * 0.6);
      positionTrajectory.initialize();
      positionTrajectory.showVisualization();

      initialOrientation.setToZero(frameToControl);
      initialOrientation.changeFrame(worldFrame);

      orientationTrajectory.setInitialConditions(initialOrientation, initialAngularVelocity);
      orientationTrajectory.setFinalConditions(finalOrientation, finalAngularVelocity);
      orientationTrajectory.setNumberOfRevolutions(0);
      orientationTrajectory.setTrajectoryTime(timeInterval.getDuration());
      orientationTrajectory.initialize();
   }

   @Override
   public void doAction(double timeInState)
   {
      if(timeInState < currentStepCommand.getTimeInterval().getDuration() * 0.2 || timeInState > currentStepCommand.getTimeInterval().getDuration() * 0.8)
      {
         supportState.doAction(timeInState);
      }
      
      else
      {
         if(notInitialized.getBooleanValue())
         {
            updateTrajectory();
            notInitialized.set(false);
            controllerToolbox.getFootContactState(robotQuadrant).clear();
         }
         positionTrajectory.compute(timeInState);
         positionTrajectory.getLinearData(desiredFootPosition, desiredFootLinearVelocity, desiredFootLinearAcceleration);
         
         orientationTrajectory.compute(timeInState);
         orientationTrajectory.getAngularData(desiredFootOrientation, desiredFootAngularVelocity, desiredFootAngularAcceleration);
         
         feedbackControlCommand.setWeightsForSolver(parameters.getSoleOrientationWeights(), parameters.getSolePositionWeights());
         feedbackControlCommand.setPositionGains(parameters.getSolePositionGains());
         feedbackControlCommand.setOrientationGains(parameters.getSoleOrientationGains());
         feedbackControlCommand.setSelectionMatrix(computeSpatialSelectionMatrix());
         feedbackControlCommand.changeFrameAndSet(desiredFootPosition, desiredFootLinearVelocity);
         feedbackControlCommand.changeFrameAndSet(desiredFootOrientation, desiredFootAngularVelocity);
         feedbackControlCommand.changeFrameAndSetFeedForward(desiredFootAngularAcceleration, desiredFootLinearAcceleration);
      }
     
   }

   private SelectionMatrix6D computeSpatialSelectionMatrix()
   {
      spatialSelectionMatrix.selectAngularX(controlAngularX.getBooleanValue());
      spatialSelectionMatrix.selectAngularY(controlAngularY.getBooleanValue());
      spatialSelectionMatrix.selectAngularZ(controlAngularZ.getBooleanValue());
//      spatialSelectionMatrix.getAngularPart().setSelectionFrame(worldFrame);

      spatialSelectionMatrix.selectLinearX(controlLinearX.getBooleanValue());
      spatialSelectionMatrix.selectLinearY(controlLinearY.getBooleanValue());
      spatialSelectionMatrix.selectLinearZ(controlLinearZ.getBooleanValue());

      return spatialSelectionMatrix;
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      QuadrupedFootControlModule.FootEvent eventToReturn = null;
      if (positionTrajectory.isDone() && timeInState > 0.2)
      {
         eventToReturn = QuadrupedFootControlModule.FootEvent.LOADED;
      }
      if (eventToReturn != null && stepTransitionCallback != null)
      {
         stepTransitionCallback.onTouchDown(robotQuadrant);
      }

      return eventToReturn;
   }

   @Override
   public void onExit()
   {
      positionTrajectory.hideVisualization();
      controllerToolbox.getFootContactState(robotQuadrant).setFullyConstrained();
   }
   
   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      if(notInitialized.getBooleanValue() || positionTrajectory.isDone())
      {
         return supportState.getInverseDynamicsCommand();
      }
      return null;
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return virtualForceCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      if(notInitialized.getBooleanValue() || positionTrajectory.isDone())
      {
         return supportState.getFeedbackControlCommand();
      }
      return feedbackControlCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return feedbackControlCommand;
   }
}

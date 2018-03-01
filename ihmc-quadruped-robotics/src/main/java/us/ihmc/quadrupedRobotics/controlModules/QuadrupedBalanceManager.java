package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.*;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlanner;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Beige;

public class QuadrupedBalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble robotTimestamp;
   private final double gravity;
   private final double mass;

   private final ReferenceFrame supportFrame;

   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrupedComPositionController comPositionController;
   private final QuadrupedComPositionController.Setpoints comPositionControllerSetpoints;

   private final DCMPlanner dcmPlanner;

   private final QuadrupedPostureInputProviderInterface postureProvider;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter comPositionProportionalGainsParameter = parameterFactory.createDoubleArray("comPositionProportionalGains", 0, 0, 5000);
   private final DoubleArrayParameter comPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("comPositionDerivativeGains", 0, 0, 750);
   private final DoubleArrayParameter comPositionIntegralGainsParameter = parameterFactory.createDoubleArray("comPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter comPositionMaxIntegralErrorParameter = parameterFactory.createDouble("comPositionMaxIntegralError", 0);
   private final DoubleArrayParameter dcmPositionProportionalGainsParameter = parameterFactory.createDoubleArray("dcmPositionProportionalGains", 1, 1, 0);
   private final DoubleArrayParameter dcmPositionDerivativeGainsParameter = parameterFactory.createDoubleArray("dcmPositionDerivativeGains", 0, 0, 0);
   private final DoubleArrayParameter dcmPositionIntegralGainsParameter = parameterFactory.createDoubleArray("dcmPositionIntegralGains", 0, 0, 0);
   private final DoubleParameter dcmPositionMaxIntegralErrorParameter = parameterFactory.createDouble("dcmPositionMaxIntegralError", 0);
   private final DoubleParameter vrpPositionRateLimitParameter = parameterFactory.createDouble("vrpPositionRateLimit", Double.MAX_VALUE);
   private final DoubleParameter comPositionGravityCompensationParameter = parameterFactory.createDouble("comPositionGravityCompensation", 1);
   private final DoubleParameter dcmPositionStepAdjustmentGainParameter = parameterFactory.createDouble("dcmPositionStepAdjustmentGain", 1.5);

   private final YoFrameVector instantaneousStepAdjustment = new YoFrameVector("instantaneousStepAdjustment", worldFrame, registry);
   private final YoFrameVector accumulatedStepAdjustment = new YoFrameVector("accumulatedStepAdjustment", worldFrame, registry);

   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();
   private final YoFramePoint yoFinalDesiredDCM;


   private final QuadrupedStepCrossoverProjection crossoverProjection;
   private final GroundPlaneEstimator groundPlaneEstimator;

   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final RecyclingArrayList<QuadrupedStep> adjustedActiveSteps;

   private final QuadrantDependentList<FramePoint3D> currentSolePositions;
   private final FramePoint3D tempPoint = new FramePoint3D();

   public QuadrupedBalanceManager(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedPostureInputProviderInterface postureProvider,
                                  YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.postureProvider = postureProvider;

      QuadrupedRuntimeEnvironment runtimeEnvironment = controllerToolbox.getRuntimeEnvironment();
      robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      gravity = 9.81;
      mass = controllerToolbox.getRuntimeEnvironment().getFullRobotModel().getTotalMass();

      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();

      yoFinalDesiredDCM = new YoFramePoint("finalDesiredDCM", worldFrame, registry);

      currentSolePositions = controllerToolbox.getTaskSpaceEstimates().getSolePositions();

      supportFrame = controllerToolbox.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();


      dcmPlanner = new DCMPlanner(gravity, postureProvider.getComPositionInput().getZ(), robotTimestamp, supportFrame, currentSolePositions, registry);

      linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();

      ReferenceFrame comZUpFrame = controllerToolbox.getReferenceFrames().getCenterOfMassZUpFrame();
      dcmPositionEstimator = controllerToolbox.getDcmPositionEstimator();
      dcmPositionController = new DivergentComponentOfMotionController(comZUpFrame, runtimeEnvironment.getControlDT(), linearInvertedPendulumModel, registry, runtimeEnvironment.getGraphicsListRegistry());
      comPositionController = new QuadrupedComPositionController(comZUpFrame, runtimeEnvironment.getControlDT(), registry);
      comPositionControllerSetpoints = new QuadrupedComPositionController.Setpoints();

      crossoverProjection = new QuadrupedStepCrossoverProjection(controllerToolbox.getReferenceFrames().getBodyZUpFrame(), registry);

      adjustedActiveSteps = new RecyclingArrayList<>(10, new GenericTypeBuilder<QuadrupedStep>()
      {
         @Override
         public QuadrupedStep newInstance()
         {
            return new QuadrupedStep();
         }
      });
      adjustedActiveSteps.clear();

      if (yoGraphicsListRegistry != null)
         setupGraphics(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private void setupGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      String graphicsListName = getClass().getSimpleName();

      YoGraphicsList graphicsList = new YoGraphicsList(graphicsListName);
      ArtifactList artifactList = new ArtifactList(graphicsListName);

      YoGraphicPosition finalDesiredCapturePointViz = new YoGraphicPosition("Final Desired Capture Point", yoFinalDesiredDCM, 0.01, Beige(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);

      graphicsList.add(finalDesiredCapturePointViz);
      artifactList.add(finalDesiredCapturePointViz.createArtifact());

      yoGraphicsListRegistry.registerYoGraphicsList(graphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void clearStepSequence()
   {
      dcmPlanner.clearStepSequence();
   }

   public void addStepToSequence(QuadrupedTimedStep step)
   {
      dcmPlanner.addStepToSequence(step);
   }

   public void addStepsToSequence(List<? extends QuadrupedTimedStep> steps)
   {
      dcmPlanner.addStepsToSequence(steps);
   }

   public void initialize(FramePoint3D comPosition, FrameVector3D comVelocity)
   {
      // update model
      linearInvertedPendulumModel.setComHeight(postureProvider.getComPositionInput().getZ());

      // update dcm estimate
      dcmPositionEstimator.compute(dcmPositionEstimate, comVelocity);

      dcmPositionController.initializeSetpoint(dcmPositionEstimate);
      dcmPositionController.reset();
      comPositionControllerSetpoints.initialize(comPosition);
      comPositionController.reset();

      // initialize timed contact sequence
      accumulatedStepAdjustment.setToZero();
   }

   public void initializeDcmSetpoints(QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
   {
      dcmPlanner.initializeDcmSetpoints(taskSpaceControllerSettings, dcmPositionEstimate);
   }

   private void computeDcmTrajectory(QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
   {
      dcmPlanner.setCoMHeight(linearInvertedPendulumModel.getComHeight());
      dcmPlanner.computeDcmTrajectory(taskSpaceControllerSettings);
   }

   public void compute(FrameVector3D linearMomentumRateOfChangeToPack, QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
   {
      updateGains();

      QuadrupedTaskSpaceEstimates taskSpaceEstimates = controllerToolbox.getTaskSpaceEstimates();
      // update model
      linearInvertedPendulumModel.setComHeight(postureProvider.getComPositionInput().getZ());

      // update dcm estimate
      dcmPositionEstimator.compute(dcmPositionEstimate, taskSpaceEstimates.getComVelocity());

      computeDcmTrajectory(taskSpaceControllerSettings);

      // update desired horizontal com forces
      dcmPlanner.computeDcmSetpoints(dcmPositionController.getDCMPositionSetpoint(), dcmPositionController.getDCMVelocitySetpoint());
      dcmPositionController.compute(linearMomentumRateOfChangeToPack, dcmPositionEstimate, dcmPositionController.getDCMPositionSetpoint(), dcmPositionController.getDCMVelocitySetpoint());
      linearMomentumRateOfChangeToPack.changeFrame(supportFrame);

      // update desired com position, velocity, and vertical force
      comPositionControllerSetpoints.getComPosition().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComPosition().set(postureProvider.getComPositionInput());
      comPositionControllerSetpoints.getComVelocity().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComVelocity().set(postureProvider.getComVelocityInput());
      comPositionControllerSetpoints.getComForceFeedforward().changeFrame(supportFrame);
      comPositionControllerSetpoints.getComForceFeedforward().set(linearMomentumRateOfChangeToPack);
      comPositionControllerSetpoints.getComForceFeedforward().setZ(comPositionGravityCompensationParameter.get() * mass * gravity);
      comPositionController.compute(linearMomentumRateOfChangeToPack, comPositionControllerSetpoints, taskSpaceEstimates);
   }

   private void updateGains()
   {
      dcmPositionController.setVrpPositionRateLimit(vrpPositionRateLimitParameter.get());
      dcmPositionController.getGains().setProportionalGains(dcmPositionProportionalGainsParameter.get());
      dcmPositionController.getGains().setIntegralGains(dcmPositionIntegralGainsParameter.get(), dcmPositionMaxIntegralErrorParameter.get());
      dcmPositionController.getGains().setDerivativeGains(dcmPositionDerivativeGainsParameter.get());

      comPositionController.getGains().setProportionalGains(comPositionProportionalGainsParameter.get());
      comPositionController.getGains().setIntegralGains(comPositionIntegralGainsParameter.get(), comPositionMaxIntegralErrorParameter.get());
      comPositionController.getGains().setDerivativeGains(comPositionDerivativeGainsParameter.get());
   }

   public void completedStep()
   {
      accumulatedStepAdjustment.add(instantaneousStepAdjustment);
      accumulatedStepAdjustment.setZ(0);
   }

   public RecyclingArrayList<QuadrupedStep> computeStepAdjustment(ArrayList<YoQuadrupedTimedStep> activeSteps)
   {
      adjustedActiveSteps.clear();
      if (robotTimestamp.getDoubleValue() > dcmPlanner.getFinalTime())
      {
         // compute step adjustment for ongoing steps (proportional to dcm tracking error)
         FramePoint3D dcmPositionSetpoint = dcmPositionController.getDCMPositionSetpoint();
         dcmPositionSetpoint.changeFrame(instantaneousStepAdjustment.getReferenceFrame());
         dcmPositionEstimate.changeFrame(instantaneousStepAdjustment.getReferenceFrame());

         instantaneousStepAdjustment.sub(dcmPositionEstimate, dcmPositionSetpoint);
         instantaneousStepAdjustment.scale(dcmPositionStepAdjustmentGainParameter.get());
         instantaneousStepAdjustment.setZ(0);

         // adjust nominal step goal positions in foot state machine
         for (int i = 0; i < activeSteps.size(); i++)
         {
            YoQuadrupedTimedStep activeStep = activeSteps.get(i);
            QuadrupedStep adjustedStep = adjustedActiveSteps.add();
            adjustedStep.set(activeStep);

            RobotQuadrant robotQuadrant = activeStep.getRobotQuadrant();
            activeStep.getGoalPosition(tempPoint);
            tempPoint.changeFrame(worldFrame);
            tempPoint.add(instantaneousStepAdjustment);
            crossoverProjection.project(tempPoint, currentSolePositions, robotQuadrant);
            groundPlaneEstimator.projectZ(tempPoint);
            adjustedStep.setGoalPosition(tempPoint);
         }
      }

      return adjustedActiveSteps;
   }

   public FrameVector3DReadOnly getAccumulatedStepAdjustment()
   {
      return accumulatedStepAdjustment;
   }
}

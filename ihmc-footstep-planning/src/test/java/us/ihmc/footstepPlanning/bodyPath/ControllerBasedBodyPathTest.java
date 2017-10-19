package us.ihmc.footstepPlanning.bodyPath;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang.mutable.MutableInt;

import com.google.common.util.concurrent.AtomicDouble;

import net.java.games.input.Component;
import net.java.games.input.Event;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.AlwaysValidNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.DistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlan;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ControllerBasedBodyPathTest
{
   private final AtomicDouble xValue = new AtomicDouble();
   private final AtomicDouble yValue = new AtomicDouble();
   private final AtomicDouble yawValue = new AtomicDouble();

   public ControllerBasedBodyPathTest() throws JoystickNotFoundException
   {
      Joystick joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
      Component forward = joystick.findComponent(XBoxOneMapping.LEFT_STICK_Y.getIdentifier());
      Component sideway = joystick.findComponent(XBoxOneMapping.LEFT_STICK_X.getIdentifier());
      Component turn = joystick.findComponent(XBoxOneMapping.RIGHT_STICK_X.getIdentifier());
      joystick.addJoystickEventListener(new JoystickEventListener()
      {
         @Override
         public void processEvent(Event event)
         {
            if (event.getComponent().equals(forward))
            {
               xValue.set(-event.getValue());
            }
            if (event.getComponent().equals(sideway))
            {
               yValue.set(-event.getValue());
            }
            if (event.getComponent().equals(turn))
            {
               yawValue.set(-event.getValue());
            }
         }
      });

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      Robot dummy = new Robot("Dummy");
      dummy.setController(new Controller(graphicsListRegistry));

      SimulationConstructionSet scs = new SimulationConstructionSet(dummy);
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(graphics3DObject);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setCameraPosition(-0.01, 0.0, 10.0);
      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.startOnAThread();
      scs.play();
   }

   private class Controller extends SimpleRobotController
   {
      private final YoDouble x = new YoDouble("x", registry);
      private final YoDouble y = new YoDouble("y", registry);

      private static final int numberOfPoints = 5;
      private final List<YoFramePoint> points = new ArrayList<>();

      private final List<Point2D> waypoints = new ArrayList<>();
      private final WaypointDefinedBodyPathPlan bodyPath = new WaypointDefinedBodyPathPlan();

      private final FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();
      private final AStarFootstepPlanner planner = createBodyPathBasedPlanner(registry, parameters, bodyPath);

      private static final int stepsPerSide = 10;
      private static final int steps = 15;
      private final SideDependentList<List<YoFramePose>> yoSteps = new SideDependentList<>();

      private final YoPolynomial xPoly = new YoPolynomial("xPoly", 4, registry);
      private final YoPolynomial yPoly = new YoPolynomial("yPoly", 4, registry);

      public Controller(YoGraphicsListRegistry graphicsListRegistry)
      {
         for (int i = 0; i < numberOfPoints; i++)
         {
            YoFramePoint yoPoint = new YoFramePoint("Position" + i, ReferenceFrame.getWorldFrame(), registry);
            YoGraphicPosition position = new YoGraphicPosition("Position" + i, yoPoint, 0.02, YoAppearance.Blue());
            points.add(yoPoint);
            graphicsListRegistry.registerYoGraphic("BodyPath", position);
         }

         YoFrameConvexPolygon2d yoDefaultFootPolygon = new YoFrameConvexPolygon2d("DefaultFootPolygon", ReferenceFrame.getWorldFrame(), 4, registry);
         yoDefaultFootPolygon.setConvexPolygon2d(PlanningTestTools.createDefaultFootPolygon());

         for (RobotSide side : RobotSide.values)
         {
            AppearanceDefinition appearance = side == RobotSide.RIGHT ? YoAppearance.Green() : YoAppearance.Red();
            ArrayList<YoFramePose> poses = new ArrayList<>();
            for (int i = 0; i < stepsPerSide; i++)
            {
               YoFramePose yoFootstepPose = new YoFramePose("footPose" + side.getCamelCaseName() + i, ReferenceFrame.getWorldFrame(), registry);
               YoGraphicPolygon footstepViz = new YoGraphicPolygon("footstep" + side.getCamelCaseName() + i, yoDefaultFootPolygon, yoFootstepPose, 1.0,
                                                                   appearance);
               poses.add(yoFootstepPose);
               yoFootstepPose.setToNaN();
               graphicsListRegistry.registerYoGraphic("viz", footstepViz);
            }
            yoSteps.put(side, poses);
         }

         planner.setWeight(1.0);
         planner.setTimeout(1.0);
      }

      @Override
      public void doControl()
      {
         double newX = Math.max(0.0, xValue.get());
         double newY = MathTools.clamp(yValue.get(), 0.5);

         if (newX != x.getDoubleValue() || newY != y.getDoubleValue())
         {
            x.set(newX);
            y.set(newY);

            double deadband = 0.2;
            boolean smallTranslation = Math.sqrt(newX * newX + newY * newY) < deadband;
            if (smallTranslation)
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  for (int i = 0; i < stepsPerSide; i++)
                  {
                     yoSteps.get(robotSide).get(i).setToNaN();
                  }
               }
               for (int i = 0; i < numberOfPoints; i++)
               {
                  points.get(i).setToNaN();
               }
               return;
            }

            waypoints.clear();
            xPoly.setQuadratic(0.0, 1.0, 0.0, 0.2, x.getDoubleValue());
            yPoly.setQuadratic(0.0, 1.0, 0.0, 0.0, y.getDoubleValue());
            for (int i = 0; i < numberOfPoints; i++)
            {
               double percent = (double) i / (double) (numberOfPoints - 1);
               xPoly.compute(percent);
               yPoly.compute(percent);
               Point2D point2d = new Point2D(xPoly.getPosition(), yPoly.getPosition());
               waypoints.add(point2d);
            }

            bodyPath.setWaypoints(waypoints);
            bodyPath.compute(null, null);

            Pose2D pose = new Pose2D();
            for (int i = 0; i < numberOfPoints; i++)
            {
               double percent = (double) i / (double) (numberOfPoints - 1);
               bodyPath.getPointAlongPath(percent, pose);
               points.get(i).set(pose.getX(), pose.getY(), 0.0);
            }

            Pose2D startPose = new Pose2D();
            bodyPath.getPointAlongPath(0.0, startPose);
            Pose2D finalPose = new Pose2D();
            bodyPath.getPointAlongPath(1.0, finalPose);

            FramePose initialMidFootPose = new FramePose();
            initialMidFootPose.setX(startPose.getX());
            initialMidFootPose.setY(startPose.getY());
            initialMidFootPose.setYawPitchRoll(0.0, 0.0, 0.0);
            PoseReferenceFrame midFootFrame = new PoseReferenceFrame("InitialMidFootFrame", initialMidFootPose);

            RobotSide initialStanceFootSide = newY > 0.0 ? RobotSide.RIGHT : RobotSide.LEFT;
            FramePose initialStanceFootPose = new FramePose(midFootFrame);
            initialStanceFootPose.setY(initialStanceFootSide.negateIfRightSide(parameters.getIdealFootstepWidth() / 2.0));
            initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());

            FramePose goalPose = new FramePose();
            goalPose.setX(finalPose.getX());
            goalPose.setY(finalPose.getY());
            goalPose.setYawPitchRoll(finalPose.getYaw(), 0.0, 0.0);

            FootstepPlannerGoal goal = new FootstepPlannerGoal();
            goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
            goal.setGoalPoseBetweenFeet(goalPose);
            goal.setXYGoal(new Point2D(goalPose.getX(), goalPose.getY()), 0.5);

            planner.setInitialStanceFoot(initialStanceFootPose, initialStanceFootSide);
            planner.setGoal(goal);
            FootstepPlanningResult result = planner.plan();

            if (result.validForExecution())
            {
               FootstepPlan footstepPlan = planner.getPlan();
               int stepsToVisualize = Math.min(footstepPlan.getNumberOfSteps(), steps);
               SideDependentList<MutableInt> counts = new SideDependentList<>(new MutableInt(0), new MutableInt(0));

               for (int i = 0; i < stepsToVisualize; i++)
               {
                  SimpleFootstep footstep = footstepPlan.getFootstep(i);
                  FramePose footstepPose = new FramePose();
                  footstep.getSoleFramePose(footstepPose);

                  RobotSide robotSide = footstep.getRobotSide();
                  MutableInt sideCount = counts.get(robotSide);
                  YoFramePose stepPose = yoSteps.get(robotSide).get(sideCount.intValue());
                  stepPose.set(footstepPose);
                  sideCount.increment();
               }

               for (RobotSide robotSide : RobotSide.values)
               {
                  MutableInt sideCount = counts.get(robotSide);
                  for (int i = sideCount.intValue(); i < stepsPerSide; i++)
                  {
                     yoSteps.get(robotSide).get(i).setToNaN();
                  }
               }
            }
            else
            {
               PrintTools.info("Failed: " + result);
            }
         }
      }

      private AStarFootstepPlanner createBodyPathBasedPlanner(YoVariableRegistry registry, FootstepPlannerParameters parameters,
                                                              WaypointDefinedBodyPathPlan bodyPath)
      {
         FootstepNodeChecker nodeChecker = new AlwaysValidNodeChecker();
         CostToGoHeuristics heuristics = new BodyPathHeuristics(registry, parameters, bodyPath);
//         CostToGoHeuristics heuristics = new DistanceAndYawBasedHeuristics(parameters, registry);
         FootstepNodeExpansion nodeExpansion = new ParameterBasedNodeExpansion(parameters);
//         FootstepNodeExpansion nodeExpansion = new SimpleSideBasedExpansion(parameters);
         FootstepCost stepCostCalculator = new DistanceAndYawBasedCost(parameters);
         FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper();
         AStarFootstepPlanner planner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, nodeExpansion, stepCostCalculator, snapper, registry);
         return planner;
      }
   }

   public static void main(String[] args) throws JoystickNotFoundException
   {
      PrintTools.info("Controller Demo.");
      new ControllerBasedBodyPathTest();
   }
}
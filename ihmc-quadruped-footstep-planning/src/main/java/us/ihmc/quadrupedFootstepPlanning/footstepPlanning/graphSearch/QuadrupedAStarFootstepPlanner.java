package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch;

import org.apache.commons.math3.util.Precision;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.QuadrupedFootstepPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.*;

public class QuadrupedAStarFootstepPlanner implements QuadrupedFootstepPlanner
{
   private static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final FootstepPlannerParameters parameters;

   private SideDependentList<FootstepNode> goalNodes;
   private HashSet<FootstepNode> expandedNodes;
   private PriorityQueue<FootstepNode> stack;
   private FootstepNode startNode;
   private FootstepNode endNode;

   private PlanarRegionsList planarRegionsList;

   private final FramePose3D goalPoseInWorld = new FramePose3D();

   private final FootstepGraph graph;
   private final FootstepNodeChecker nodeChecker;
   private final QuadrupedFootstepPlannerListener listener;
   private final CostToGoHeuristics heuristics;
   private final FootstepNodeExpansion nodeExpansion;
   private final FootstepCost stepCostCalculator;
   private final FootstepNodeSnapper snapper;

   private final ArrayList<StartAndGoalListener> startAndGoalListeners = new ArrayList<>();

   private final YoDouble timeout = new YoDouble("footstepPlannerTimeout", registry);
   private final YoDouble planningTime = new YoDouble("PlanningTime", registry);
   private final YoLong numberOfExpandedNodes = new YoLong("NumberOfExpandedNodes", registry);
   private final YoDouble percentRejectedNodes = new YoDouble("PercentRejectedNodes", registry);
   private final YoLong iterationCount = new YoLong("IterationCount", registry);

   private final YoBoolean initialize = new YoBoolean("initialize", registry);

   private final YoBoolean validGoalNode = new YoBoolean("validGoalNode", registry);
   private final YoBoolean abortPlanning = new YoBoolean("abortPlanning", registry);

   public QuadrupedAStarFootstepPlanner(FootstepPlannerParameters parameters, FootstepNodeChecker nodeChecker, CostToGoHeuristics heuristics,
                                        FootstepNodeExpansion expansion, FootstepCost stepCostCalculator, FootstepNodeSnapper snapper, YoVariableRegistry parentRegistry)
   {
      this(parameters, nodeChecker, heuristics, expansion, stepCostCalculator, snapper, null, parentRegistry);
   }

   public QuadrupedAStarFootstepPlanner(FootstepPlannerParameters parameters, FootstepNodeChecker nodeChecker, CostToGoHeuristics heuristics,
                                        FootstepNodeExpansion nodeExpansion, FootstepCost stepCostCalculator, FootstepNodeSnapper snapper,
                                        BipedalFootstepPlannerListener listener, YoVariableRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.nodeChecker = nodeChecker;
      this.heuristics = heuristics;
      this.nodeExpansion = nodeExpansion;
      this.stepCostCalculator = stepCostCalculator;
      this.listener = listener;
      this.snapper = snapper;
      this.graph = new FootstepGraph();
      timeout.set(Double.POSITIVE_INFINITY);
      this.initialize.set(true);

      parentRegistry.addChild(registry);
   }

   public void addStartAndGoalListener(StartAndGoalListener startAndGoalListener)
   {
      startAndGoalListeners.add(startAndGoalListener);
   }

   @Override
   public void setTimeout(double timeoutInSeconds)
   {
      timeout.set(timeoutInSeconds);
   }

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      if (side == null)
      {
         if (debug)
            PrintTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         side = defaultStartNodeSide;
      }
      startNode = new FootstepNode(stanceFootPose.getX(), stanceFootPose.getY(), stanceFootPose.getYaw(), side);
      RigidBodyTransform startNodeSnapTransform = FootstepNodeSnappingTools.computeSnapTransform(startNode, stanceFootPose);
      snapper.addSnapData(startNode, new FootstepNodeSnapData(startNodeSnapTransform));
      nodeChecker.addStartNode(startNode, startNodeSnapTransform);

      startAndGoalListeners.parallelStream().forEach(listener -> listener.setInitialPose(stanceFootPose));
   }

   @Override
   public void setGoal(QuadrupedFootstepPlannerGoal goal)
   {
      checkGoalType(goal);

      goalNodes = new SideDependentList<>();

      SideDependentList<FramePose3D> goalPoses = new SideDependentList<>();

      if (goal.getFootstepPlannerGoalType().equals(FootstepPlannerGoalType.POSE_BETWEEN_FEET))
      {
         FramePose3D goalPose = goal.getGoalPoseBetweenFeet();
         ReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);
         for (RobotSide side : RobotSide.values)
         {
            FramePose3D goalNodePose = new FramePose3D(goalFrame);
            goalNodePose.setY(side.negateIfRightSide(parameters.getIdealFootstepWidth() / 2.0));
            goalNodePose.changeFrame(goalPose.getReferenceFrame());
            FootstepNode goalNode = new FootstepNode(goalNodePose.getX(), goalNodePose.getY(), goalNodePose.getYaw(), side);
            goalNodes.put(side, goalNode);

            goalNodePose.changeFrame(ReferenceFrame.getWorldFrame());
            goalPoses.put(side, goalNodePose);
         }
      }
      else if (goal.getFootstepPlannerGoalType().equals(FootstepPlannerGoalType.DOUBLE_FOOTSTEP))
      {
         SideDependentList<SimpleFootstep> goalSteps = goal.getDoubleFootstepGoal();
         for (RobotSide side : RobotSide.values)
         {
            FramePose3D goalNodePose = new FramePose3D();
            goalSteps.get(side).getSoleFramePose(goalNodePose);
            FootstepNode goalNode = new FootstepNode(goalNodePose.getX(), goalNodePose.getY(), goalNodePose.getYaw(), side);
            goalNodes.put(side, goalNode);

            goalNodePose.changeFrame(ReferenceFrame.getWorldFrame());
            goalPoses.put(side, goalNodePose);
         }
      }

      goalPoseInWorld.interpolate(goalPoses.get(RobotSide.LEFT), goalPoses.get(RobotSide.RIGHT), 0.5);
      startAndGoalListeners.parallelStream().forEach(listener -> listener.setGoalPose(goalPoseInWorld));
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      nodeChecker.setPlanarRegions(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      if (initialize.getBooleanValue())
      {
         boolean success = initialize();
         initialize.set(false);
         if (!success)
            return FootstepPlanningResult.PLANNER_FAILED;
      }

      if (debug)
         PrintTools.info("A* planner has initialized");

      if (!planInternal())
         return FootstepPlanningResult.PLANNER_FAILED;

      FootstepPlanningResult result = checkResult();

      if (result.validForExecution() && listener != null)
         listener.plannerFinished(null);

      if (debug)
      {
         PrintTools.info("A* Footstep planning statistics for " + result);
         System.out.println("   Finished planning after " + Precision.round(planningTime.getDoubleValue(), 2) + " seconds.");
         System.out.println("   Expanded each node to an average of " + numberOfExpandedNodes.getLongValue() + " children nodes.");
         System.out.println("   Planning took a total of " + iterationCount.getLongValue() + " iterations.");
         System.out.println("   During the planning " + percentRejectedNodes.getDoubleValue() + "% of nodes were rejected as invalid.");
         System.out.println("   Goal was : " + goalPoseInWorld);
      }

      initialize.set(true);
      return result;
   }

   @Override
   public FootstepPlan getPlan()
   {
      if (endNode == null || !graph.doesNodeExist(endNode))
         return null;

      FootstepPlan plan = new FootstepPlan();
      List<FootstepNode> path = graph.getPathFromStart(endNode);
      for (int i = 1; i < path.size(); i++)
      {
         RobotSide robotSide = path.get(i).getRobotSide();

         RigidBodyTransform footstepPose = new RigidBodyTransform();
         footstepPose.setRotationYawAndZeroTranslation(path.get(i).getYaw());
         footstepPose.setTranslationX(path.get(i).getX());
         footstepPose.setTranslationY(path.get(i).getY());

         FootstepNodeSnapData snapData = snapper.snapFootstepNode(path.get(i));
         RigidBodyTransform snapTransform = snapData.getSnapTransform();
         snapTransform.transform(footstepPose);
         plan.addFootstep(robotSide, new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPose));

         ConvexPolygon2D foothold = snapData.getCroppedFoothold();
         if (!foothold.isEmpty())
            plan.getFootstep(i - 1).setFoothold(foothold);
      }

      plan.setLowLevelPlanGoal(goalPoseInWorld);

      return plan;
   }

   @Override
   public double getPlanningDuration()
   {
      return planningTime.getDoubleValue();
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizon)
   {
   }

   private boolean initialize()
   {
      if (startNode == null)
         throw new NullPointerException("Need to set initial conditions before planning.");
      if (goalNodes == null)
         throw new NullPointerException("Need to set goal before planning.");

      abortPlanning.set(false);

      if (planarRegionsList != null)
         checkStartHasPlanarRegion();

      graph.initialize(startNode);
      NodeComparator nodeComparator = new NodeComparator(graph, goalNodes, heuristics);
      stack = new PriorityQueue<>(nodeComparator);

      validGoalNode.set(true);
      for (RobotSide robotSide : RobotSide.values)
      {
         boolean validGoalNode = nodeChecker.isNodeValid(goalNodes.get(robotSide), null);
         if (!validGoalNode && !parameters.getReturnBestEffortPlan())
         {
            if (debug)
               PrintTools.info("Goal node isn't valid. To plan without a valid goal node, best effort planning must be enabled");

            return false;
         }

         this.validGoalNode.set(validGoalNode && this.validGoalNode.getBooleanValue());
      }

      stack.add(startNode);
      expandedNodes = new HashSet<>();
      endNode = null;

      if (listener != null)
      {
         listener.addNode(startNode, null);
         listener.tickAndUpdate();
      }

      return true;
   }

   private void checkStartHasPlanarRegion()
   {
      Point3D startPoint = new Point3D(startNode.getX(), startNode.getY(), 0.0);
      Point3DReadOnly startPos = PlanarRegionTools
            .projectPointToPlanesVertically(startPoint, snapper.getOrCreateSteppableRegions(startNode.getRoundedX(), startNode.getRoundedY()));

      if (startPos == null)
      {
         if (debug)
            PrintTools.info("adding plane at start foot");
         addPlanarRegionAtZeroHeight(startNode.getX(), startNode.getY());
      }
   }

   private void addPlanarRegionAtZeroHeight(double xLocation, double yLocation)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.3, 0.3);
      polygon.addVertex(-0.3, 0.3);
      polygon.addVertex(0.3, -0.3);
      polygon.addVertex(-0.3, -0.25);
      polygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(new AxisAngle(), new Vector3D(xLocation, yLocation, 0.0)), polygon);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   @Override
   public void cancelPlanning()
   {
      if (debug)
         PrintTools.info("Cancel has been requested.");
      abortPlanning.set(true);
   }

   public void requestInitialize()
   {
      initialize.set(true);
   }

   private boolean planInternal()
   {
      long planningStartTime = System.nanoTime();

      long rejectedNodesCount = 0;
      long expandedNodesCount = 0;
      long iterations = 0;

      while (!stack.isEmpty())
      {
         if (initialize.getBooleanValue())
         {
            boolean success = initialize();
            rejectedNodesCount = 0;
            expandedNodesCount = 0;
            iterations = 0;
            initialize.set(false);
            if (!success)
               return false;
         }

         iterations++;

         FootstepNode nodeToExpand = stack.poll();
         if (expandedNodes.contains(nodeToExpand))
            continue;
         expandedNodes.add(nodeToExpand);

         if (checkAndHandleNodeAtGoal(nodeToExpand))
            break;

         checkAndHandleBestEffortNode(nodeToExpand);

         HashSet<FootstepNode> neighbors = nodeExpansion.expandNode(nodeToExpand);
         expandedNodesCount += neighbors.size();
         for (FootstepNode neighbor : neighbors)
         {
            if (listener != null)
               listener.addNode(neighbor, nodeToExpand);

            // Checks if the footstep (center of the foot) is on a planar region
            if (!nodeChecker.isNodeValid(neighbor, nodeToExpand))
            {
               rejectedNodesCount++;
               continue;
            }

            double cost = stepCostCalculator.compute(nodeToExpand, neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, cost);

            if (!parameters.getReturnBestEffortPlan() || endNode == null || stack.comparator().compare(neighbor, endNode) < 0)
               stack.add(neighbor);
         }

         if (listener != null)
            listener.tickAndUpdate();

         long timeInNano = System.nanoTime();
         if (Conversions.nanosecondsToSeconds(timeInNano - planningStartTime) > timeout.getDoubleValue() || abortPlanning.getBooleanValue())
         {
            if (abortPlanning.getBooleanValue())
               PrintTools.info("Abort planning requested.");
            abortPlanning.set(false);
            break;
         }
      }

      long timeInNano = System.nanoTime();
      planningTime.set(Conversions.nanosecondsToSeconds(timeInNano - planningStartTime));
      percentRejectedNodes.set(100.0 * rejectedNodesCount / expandedNodesCount);
      iterationCount.set(iterations);
      numberOfExpandedNodes.set(expandedNodesCount / Math.max(iterations, 1));

      return true;
   }

   private boolean checkAndHandleNodeAtGoal(FootstepNode nodeToExpand)
   {
      if (!validGoalNode.getBooleanValue())
         return false;

      RobotSide nodeSide = nodeToExpand.getRobotSide();
      if (goalNodes.get(nodeSide).equals(nodeToExpand))
      {
         endNode = goalNodes.get(nodeSide.getOppositeSide());
         graph.checkAndSetEdge(nodeToExpand, endNode, 0.0);
         return true;
      }

      return false;
   }

   private void checkAndHandleBestEffortNode(FootstepNode nodeToExpand)
   {
      if (!parameters.getReturnBestEffortPlan())
         return;

      if (graph.getPathFromStart(nodeToExpand).size() - 1 < parameters.getMinimumStepsForBestEffortPlan())
         return;

      if (endNode == null || heuristics.compute(nodeToExpand, goalNodes.get(nodeToExpand.getRobotSide())) < heuristics
            .compute(endNode, goalNodes.get(endNode.getRobotSide())))
      {
         if (listener != null)
            listener.reportLowestCostNodeList(graph.getPathFromStart(nodeToExpand));
         endNode = nodeToExpand;
      }
   }

   private FootstepPlanningResult checkResult()
   {
      if (stack.isEmpty() && endNode == null)
         return FootstepPlanningResult.NO_PATH_EXISTS;
      if (!graph.doesNodeExist(endNode))
         return FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION;

      if (heuristics.getWeight() <= 1.0)
         return FootstepPlanningResult.OPTIMAL_SOLUTION;

      return FootstepPlanningResult.SUB_OPTIMAL_SOLUTION;
   }

   public static void checkGoalType(QuadrupedFootstepPlannerGoal goal)
   {
      FootstepPlannerGoalType supportedGoalType1 = FootstepPlannerGoalType.POSE_BETWEEN_FEET;
      FootstepPlannerGoalType supportedGoalType2 = FootstepPlannerGoalType.DOUBLE_FOOTSTEP;
      if (!goal.getFootstepPlannerGoalType().equals(supportedGoalType1) && !goal.getFootstepPlannerGoalType().equals(supportedGoalType2))
         throw new IllegalArgumentException("Planner does not support goals other than " + supportedGoalType1 + " and " + supportedGoalType2);
   }

   public static QuadrupedAStarFootstepPlanner createPlanner(FootstepPlannerParameters parameters, QuadrupedFootstepPlannerListener listener,
                                                             FootstepNodeExpansion expansion, YoVariableRegistry registry)
   {
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters);
//      FootstepNodeSnapAndWiggler postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      SimplePlanarRegionFootstepNodeSnapper postProcessingSnapper = new SimplePlanarRegionFootstepNodeSnapper(parameters);

      SnapBasedNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(parameters, snapper);
//      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(parameters, snapper, footPolygons);

      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(parameters.getCostParameters().getAStarHeuristicsWeight(), parameters);

      FootstepNodeChecker nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, bodyCollisionNodeChecker, cliffAvoider));
      nodeChecker.addPlannerListener(listener);

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludePitchAndRollCost(true);

      FootstepCost footstepCost = costBuilder.buildCost();

      QuadrupedAStarFootstepPlanner planner = new QuadrupedAStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, footstepCost, postProcessingSnapper, listener,
                                                                                registry);


      return planner;
   }
}

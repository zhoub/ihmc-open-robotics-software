package us.ihmc.footstepPlanning.graphSearch.pathPlanners;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

public abstract class WaypointsForFootstepsPlanner
{
   protected static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   protected final YoEnum<FootstepPlanningResult> yoResult;

   protected final FootstepPlannerParameters parameters;
   protected final FramePose3D bodyStartPose = new FramePose3D();
   protected final FramePose3D bodyGoalPose = new FramePose3D();

   protected final List<Point3D> waypoints = new ArrayList<>();

   protected PlanarRegionsList planarRegionsList;

   public WaypointsForFootstepsPlanner(FootstepPlannerParameters parameters, YoVariableRegistry registry)
   {
      this.parameters = parameters;

      yoResult = new YoEnum<>("pathPlanningResult", registry, FootstepPlanningResult.class);
   }

   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      if (side == null)
      {
         if (debug)
            PrintTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         side = defaultStartNodeSide;
      }

      double defaultStepWidth = parameters.getIdealFootstepWidth();
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);
      FramePoint2D bodyStartPoint = new FramePoint2D(stanceFrame);
      bodyStartPoint.setY(side.negateIfLeftSide(defaultStepWidth / 2.0));
      bodyStartPoint.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      bodyStartPose.setToZero(ReferenceFrame.getWorldFrame());
      bodyStartPose.setPosition(bodyStartPoint.getX(), bodyStartPoint.getY(), 0.0);
      bodyStartPose.setOrientationYawPitchRoll(stanceFootPose.getYaw(), 0.0, 0.0);
   }

   public void setGoal(FootstepPlannerGoal goal)
   {
      AStarFootstepPlanner.checkGoalType(goal);
      FramePose3D goalPose = goal.getGoalPoseBetweenFeet();
      bodyGoalPose.setIncludingFrame(goalPose);
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public List<? extends Point3DReadOnly> getWaypoints()
   {
      return waypoints;
   }

   public abstract FootstepPlanningResult planWaypoints();
}
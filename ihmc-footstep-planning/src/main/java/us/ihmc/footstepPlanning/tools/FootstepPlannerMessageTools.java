package us.ihmc.footstepPlanning.tools;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlannerMessageTools
{
   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket(FramePose3D initialStanceFootPose, RobotSide initialStanceSide,
                                                                                   FramePose3D goalPose)
   {
      return createFootstepPlanningRequestPacket(initialStanceFootPose, initialStanceSide, goalPose, FootstepPlannerType.PLANAR_REGION_BIPEDAL);
   }

   public static FootstepPlanningRequestPacket createFootstepPlanningRequestPacket(FramePose3D initialStanceFootPose, RobotSide initialStanceSide,
                                                                                   FramePose3D goalPose, FootstepPlannerType requestedPlannerType)
   {
      FootstepPlanningRequestPacket message = new FootstepPlanningRequestPacket();
      message.setInitialStanceRobotSide(initialStanceSide.toByte());

      FramePoint3D initialFramePoint = new FramePoint3D(initialStanceFootPose.getPosition());
      initialFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      message.getStanceFootPositionInWorld().set(new Point3D32(initialFramePoint));

      FrameQuaternion initialFrameOrientation = new FrameQuaternion(initialStanceFootPose.getOrientation());
      initialFrameOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      message.getStanceFootOrientationInWorld().set(new Quaternion32(initialFrameOrientation));

      FramePoint3D goalFramePoint = new FramePoint3D(goalPose.getPosition());
      goalFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      message.getGoalPositionInWorld().set(new Point3D32(goalFramePoint));

      FrameQuaternion goalFrameOrientation = new FrameQuaternion(goalPose.getOrientation());
      goalFrameOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      message.getGoalOrientationInWorld().set(new Quaternion32(goalFrameOrientation));

      message.setRequestedFootstepPlannerType(requestedPlannerType.toByte());
      return message;
   }

   public static void copyParametersToPacket(FootstepPlannerParametersPacket packet, FootstepPlannerParameters parameters)
   {
      if (parameters == null)
      {
         return;
      }

      packet.setCheckForBodyBoxCollisions(parameters.checkForBodyBoxCollisions());
      packet.setPerformHeuristicSearchPolicies(parameters.performHeuristicSearchPolicies());
      packet.setIdealFootstepWidth(parameters.getIdealFootstepWidth());
      packet.setIdealFootstepLength(parameters.getIdealFootstepLength());
      packet.setWiggleInsideDelta(parameters.getWiggleInsideDelta());
      packet.setMaximumStepReach(parameters.getMaximumStepReach());
      packet.setMaximumStepYaw(parameters.getMaximumStepYaw());
      packet.setMinimumStepWidth(parameters.getMinimumStepWidth());
      packet.setMinimumStepLength(parameters.getMinimumStepLength());
      packet.setMinimumStepYaw(parameters.getMinimumStepYaw());
      packet.setMaximumStepReachWhenSteppingUp(parameters.getMaximumStepReachWhenSteppingUp());
      packet.setMaximumStepZWhenSteppingUp(parameters.getMaximumStepZWhenSteppingUp());
      packet.setMaximumStepXWhenForwardAndDown(parameters.getMaximumStepXWhenForwardAndDown());
      packet.setMaximumStepZWhenForwardAndDown(parameters.getMaximumStepZWhenForwardAndDown());
      packet.setMaximumStepZ(parameters.getMaximumStepZ());
      packet.setMinimumFootholdPercent(parameters.getMinimumFootholdPercent());
      packet.setMinimumSurfaceInclineRadians(parameters.getMinimumSurfaceInclineRadians());
      packet.setWiggleIntoConvexHullOfPlanarRegions(parameters.getWiggleIntoConvexHullOfPlanarRegions());
      packet.setRejectIfCannotFullyWiggleInside(parameters.getRejectIfCannotFullyWiggleInside());
      packet.setMaximumXyWiggleDistance(parameters.getMaximumXYWiggleDistance());
      packet.setMaximumYawWiggle(parameters.getMaximumYawWiggle());
      packet.setMaximumZPenetrationOnValleyRegions(parameters.getMaximumZPenetrationOnValleyRegions());
      packet.setMaximumStepWidth(parameters.getMaximumStepWidth());
      packet.setMinimumDistanceFromCliffBottoms(parameters.getMinimumDistanceFromCliffBottoms());
      packet.setCliffHeightToAvoid(parameters.getCliffHeightToAvoid());
      packet.setReturnBestEffortPlan(parameters.getReturnBestEffortPlan());
      packet.setMinimumStepsForBestEffortPlan(parameters.getMinimumStepsForBestEffortPlan());
      packet.setBodyGroundClearance(parameters.getBodyGroundClearance());
      packet.setBodyBoxHeight(parameters.getBodyBoxHeight());
      packet.setBodyBoxDepth(parameters.getBodyBoxDepth());
      packet.setBodyBoxWidth(parameters.getBodyBoxWidth());
      packet.setBodyBoxBaseX(parameters.getBodyBoxBaseX());
      packet.setBodyBoxBaseY(parameters.getBodyBoxBaseY());
      packet.setBodyBoxBaseZ(parameters.getBodyBoxBaseZ());
      packet.setMinXClearanceFromStance(parameters.getMinXClearanceFromStance());
      packet.setMinYClearanceFromStance(parameters.getMinYClearanceFromStance());
      packet.setStepTranslationBoundingBoxScaleFactor(parameters.getStepTranslationBoundingBoxScaleFactor());

      FootstepPlannerCostParameters costParameters = parameters.getCostParameters();

      packet.getCostParameters().setUseQuadraticDistanceCost(costParameters.useQuadraticDistanceCost());
      packet.getCostParameters().setUseQuadraticHeightCost(costParameters.useQuadraticHeightCost());

      packet.getCostParameters().setAStarHeuristicsWeight(costParameters.getAStarHeuristicsWeight().getValue());
      packet.getCostParameters().setVisGraphWithAStarHeuristicsWeight(costParameters.getVisGraphWithAStarHeuristicsWeight().getValue());
      packet.getCostParameters().setDepthFirstHeuristicsWeight(costParameters.getDepthFirstHeuristicsWeight().getValue());
      packet.getCostParameters().setBodyPathBasedHeuristicsWeight(costParameters.getBodyPathBasedHeuristicsWeight().getValue());

      packet.getCostParameters().setYawWeight(costParameters.getYawWeight());
      packet.getCostParameters().setPitchWeight(costParameters.getPitchWeight());
      packet.getCostParameters().setRollWeight(costParameters.getRollWeight());
      packet.getCostParameters().setStepUpWeight(costParameters.getStepUpWeight());
      packet.getCostParameters().setStepDownWeight(costParameters.getStepDownWeight());
      packet.getCostParameters().setForwardWeight(costParameters.getForwardWeight());
      packet.getCostParameters().setLateralWeight(costParameters.getLateralWeight());
      packet.getCostParameters().setCostPerStep(costParameters.getCostPerStep());
   }

   public static void copyParametersToPacket(VisibilityGraphsParametersPacket packet, VisibilityGraphsParameters parameters)
   {
      if (parameters == null)
      {
         return;
      }

      packet.setMaxInterRegionConnectionLength(parameters.getMaxInterRegionConnectionLength());
      packet.setNormalZThresholdForAccessibleRegions(parameters.getNormalZThresholdForAccessibleRegions());
      packet.setExtrusionDistance(parameters.getObstacleExtrusionDistance());
      packet.setExtrusionDistanceIfNotTooHighToStep(parameters.getObstacleExtrusionDistanceIfNotTooHighToStep());
      packet.setTooHighToStepDistance(parameters.getTooHighToStepDistance());
      packet.setClusterResolution(parameters.getClusterResolution());
      packet.setExplorationDistanceFromStartGoal(parameters.getExplorationDistanceFromStartGoal());
      packet.setPlanarRegionMinArea(parameters.getPlanarRegionMinArea());
      packet.setPlanarRegionMinSize(parameters.getPlanarRegionMinSize());
      packet.setRegionOrthogonalAngle(parameters.getRegionOrthogonalAngle());
      packet.setSearchHostRegionEpsilon(parameters.getSearchHostRegionEpsilon());
   }
}

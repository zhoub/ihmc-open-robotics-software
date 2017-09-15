package us.ihmc.humanoidRobotics.footstep;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class Footstep
{
   public static enum FootstepType {FULL_FOOTSTEP, PARTIAL_FOOTSTEP, BAD_FOOTSTEP}

   public static final int maxNumberOfSwingWaypoints = 100;

   private RobotSide robotSide;
   private FootstepType footstepType = FootstepType.FULL_FOOTSTEP;

   private final FramePose footstepPose = new FramePose();

   private final FramePose tempPose = new FramePose();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   // TODO: nuke this.
   private static int counter = 0;
   private final PoseReferenceFrame footstepSoleFrame;

   private final List<Point2D> predictedContactPoints = new ArrayList<>();

   private final RecyclingArrayList<FramePoint3D> customPositionWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> swingTrajectory = new RecyclingArrayList<>(maxNumberOfSwingWaypoints, FrameSE3TrajectoryPoint.class);
   private double swingTrajectoryBlendDuration = 0.0;

   private final boolean trustHeight;
   private boolean scriptedFootstep;

   // foot trajectory generation
   public TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   public double swingHeight = 0;

   public Footstep(RobotSide robotSide, FramePose footstepPose, boolean trustHeight)
   {
      this(robotSide, footstepPose, trustHeight, null);
   }

   public Footstep(RobotSide robotSide)
   {
      this(robotSide, new FramePose());
   }

   public Footstep(RobotSide robotSide, FramePose footstepPose)
   {
      this(robotSide, footstepPose, true);
   }

   public Footstep(Footstep footstep)
   {
      this(footstep.robotSide, footstep.footstepPose, footstep.trustHeight);
      this.trajectoryType = footstep.trajectoryType;
      this.swingHeight = footstep.swingHeight;
      this.swingTrajectoryBlendDuration = footstep.swingTrajectoryBlendDuration;
   }

   public Footstep(RobotSide robotSide, FramePose footstepPose, boolean trustHeight, List<Point2D> predictedContactPoints)
   {
      this(robotSide, footstepPose, trustHeight, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public Footstep(RobotSide robotSide, FramePose footstepPose, boolean trustHeight, List<Point2D> predictedContactPoints,
                   TrajectoryType trajectoryType, double swingHeight)
   {
      this.robotSide = robotSide;
      this.trustHeight = trustHeight;
      this.footstepPose.setIncludingFrame(footstepPose);
      setPredictedContactPointsFromPoint2ds(predictedContactPoints);
      this.trajectoryType = trajectoryType;
      this.swingHeight = swingHeight;

      footstepSoleFrame = new PoseReferenceFrame(counter++ + "_FootstepSoleFrame", ReferenceFrame.getWorldFrame());
   }

   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public List<FramePoint3D> getCustomPositionWaypoints()
   {
      return customPositionWaypoints;
   }

   public void setCustomPositionWaypoints(RecyclingArrayList<FramePoint3D> customPositionWaypoints)
   {
      this.customPositionWaypoints.clear();
      for (int i = 0; i < customPositionWaypoints.size(); i++)
         this.customPositionWaypoints.add().set(customPositionWaypoints.get(i));
   }

   public List<FrameSE3TrajectoryPoint> getSwingTrajectory()
   {
      return swingTrajectory;
   }

   public void setSwingTrajectory(RecyclingArrayList<FrameSE3TrajectoryPoint> swingTrajectory)
   {
      this.swingTrajectory.clear();
      for (int i = 0; i < swingTrajectory.size(); i++)
         this.swingTrajectory.add().set(swingTrajectory.get(i));
   }

   public void setSwingTrajectoryBlendDuration(double swingTrajectoryBlendDuration)
   {
      this.swingTrajectoryBlendDuration = swingTrajectoryBlendDuration;
   }

   public double getSwingTrajectoryBlendDuration()
   {
      return swingTrajectoryBlendDuration;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public void setPredictedContactPointsFromPoint2ds(List<Point2D> contactPointList)
   {
      efficientlyResizeContactPointList(contactPointList);

      if ((contactPointList == null) || contactPointList.isEmpty())
      {
         if ((footstepType != FootstepType.FULL_FOOTSTEP) && (footstepType != FootstepType.BAD_FOOTSTEP))
         {
            footstepType = FootstepType.FULL_FOOTSTEP;
         }

         predictedContactPoints.clear();

         return;
      }

      footstepType = FootstepType.PARTIAL_FOOTSTEP;

      for (int i = 0; i < contactPointList.size(); i++)
      {
         Point2D point = contactPointList.get(i);
         this.predictedContactPoints.get(i).set(point.getX(), point.getY());
      }
   }

   public void setPredictedContactPointsFromFramePoint2ds(List<FramePoint2D> contactPointList)
   {
      efficientlyResizeContactPointList(contactPointList);

      if ((contactPointList == null) || contactPointList.isEmpty())
      {
         if ((footstepType != FootstepType.FULL_FOOTSTEP) && (footstepType != FootstepType.BAD_FOOTSTEP))
         {
            footstepType = FootstepType.FULL_FOOTSTEP;
         }

         predictedContactPoints.clear();

         return;
      }

      footstepType = FootstepType.PARTIAL_FOOTSTEP;

      for (int i = 0; i < contactPointList.size(); i++)
      {
         FramePoint2D point = contactPointList.get(i);
         this.predictedContactPoints.get(i).set(point.getX(), point.getY());
      }

   }

   private void efficientlyResizeContactPointList(List<?> contactPointList)
   {
      if ((contactPointList == null) || contactPointList.isEmpty())
      {
         this.predictedContactPoints.clear();

         return;
      }

      footstepType = FootstepType.PARTIAL_FOOTSTEP;

      int newSize = contactPointList.size();
      int currentSize = this.predictedContactPoints.size();

      if (currentSize > newSize)
      {
         for (int i = 0; i < currentSize - newSize; i++)
         {
            this.predictedContactPoints.remove(0);
         }

         currentSize = this.predictedContactPoints.size();
      }

      if (currentSize < newSize)
      {
         for (int i = 0; i < newSize - currentSize; i++)
         {
            this.predictedContactPoints.add(new Point2D());
         }
      }
   }

   public List<Point2D> getPredictedContactPoints()
   {
      if (predictedContactPoints.isEmpty())
      {
         return null;
      }
      return predictedContactPoints;
   }

   public void setX(double x)
   {
      footstepPose.setX(x);
   }

   public void setY(double y)
   {
      footstepPose.setY(y);
   }

   public void setZ(double z)
   {
      footstepPose.setZ(z);
   }

   public double getX()
   {
      return footstepPose.getX();
   }

   public double getY()
   {
      return footstepPose.getY();
   }

   public double getZ()
   {
      return footstepPose.getZ();
   }

   public void setPose(RigidBodyTransform transformFromAnkleToWorldFrame)
   {
      footstepPose.setToNaN(ReferenceFrame.getWorldFrame());
      footstepPose.setPose(transformFromAnkleToWorldFrame);
   }

   public void setPose(Footstep newFootstep)
   {
      footstepPose.setIncludingFrame(newFootstep.getFootstepPose());
   }

   public void setPose(FramePose footstepPose)
   {
      this.footstepPose.setIncludingFrame(footstepPose);
   }

   public void setPose(FramePoint3D position, FrameOrientation orientation)
   {
      footstepPose.setPoseIncludingFrame(position, orientation);
   }

   public void setPositionChangeOnlyXY(FramePoint2D position2d)
   {
      position2d.checkReferenceFrameMatch(footstepPose);
      setX(position2d.getX());
      setY(position2d.getY());
   }

   public boolean getTrustHeight()
   {
      return trustHeight;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public FramePose getFootstepPose()
   {
      return footstepPose;
   }

   public void getPose(FramePose poseToPack)
   {
      poseToPack.setIncludingFrame(footstepPose);
   }

   public void getPose(FramePoint3D positionToPack, FrameOrientation orientationToPack)
   {
      footstepPose.getPoseIncludingFrame(positionToPack, orientationToPack);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      footstepPose.getPositionIncludingFrame(positionToPack);
   }

   public void getPosition2d(FramePoint2D positionToPack)
   {
      footstepPose.getPosition2dIncludingFrame(positionToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      footstepPose.getOrientationIncludingFrame(orientationToPack);
   }

   public ReferenceFrame getTrajectoryFrame()
   {
      return footstepPose.getReferenceFrame();
   }

   public void setFootstepType(FootstepType footstepType)
   {
      this.footstepType = footstepType;
   }

   public FootstepType getFootstepType()
   {
      return footstepType;
   }

   public boolean isScriptedFootstep()
   {
      return scriptedFootstep;
   }

   public void setScriptedFootstep(boolean scriptedFootstep)
   {
      this.scriptedFootstep = scriptedFootstep;
   }

   public boolean epsilonEquals(Footstep otherFootstep, double epsilon)
   {
      boolean arePosesEqual = footstepPose.epsilonEquals(otherFootstep.footstepPose, epsilon);
      boolean sameRobotSide = robotSide == otherFootstep.robotSide;
      boolean isTrustHeightTheSame = trustHeight == otherFootstep.trustHeight;

      boolean sameWaypoints = customPositionWaypoints.size() == otherFootstep.customPositionWaypoints.size();
      if (sameWaypoints)
      {
         for (int i = 0; i < customPositionWaypoints.size(); i++)
         {
            FramePoint3D waypoint = customPositionWaypoints.get(i);
            FramePoint3D otherWaypoint = otherFootstep.customPositionWaypoints.get(i);
            sameWaypoints = sameWaypoints && waypoint.epsilonEquals(otherWaypoint, epsilon);
         }
      }

      boolean sameBlendDuration = MathTools.epsilonEquals(swingTrajectoryBlendDuration, otherFootstep.swingTrajectoryBlendDuration, epsilon);

      return arePosesEqual && sameRobotSide && isTrustHeightTheSame && sameWaypoints && sameBlendDuration;
   }

   @Override
   public String toString()
   {
      return "pose: " + footstepPose + " - trustHeight = " + trustHeight;
   }

   // Going to be removed soon. Is duplicate information.
   @Deprecated
   public ReferenceFrame getSoleReferenceFrame()
   {
      tempPose.setIncludingFrame(footstepPose);
      tempPose.changeFrame(footstepSoleFrame.getParent());
      footstepSoleFrame.setPoseAndUpdate(tempPose);
      return footstepSoleFrame;
   }

   public void getAnklePose(FramePose poseToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      poseToPack.setPoseIncludingFrame(footstepPose.getReferenceFrame(), tempTransform);
   }

   public void getAnklePosition(FramePoint3D positionToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      positionToPack.setIncludingFrame(footstepPose.getReferenceFrame(), tempTransform.getTranslationVector());
   }

   public void getAnklePosition2d(FramePoint2D position2dToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      double x = tempTransform.getTranslationVector().getX();
      double y = tempTransform.getTranslationVector().getY();
      position2dToPack.setIncludingFrame(footstepPose.getReferenceFrame(), x, y);
   }

   public void getAnkleOrientation(FrameOrientation orientationToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      orientationToPack.setIncludingFrame(footstepPose.getReferenceFrame(), tempTransform.getRotationMatrix());
   }

   public void setFromAnklePose(FramePose anklePose, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(anklePose.getOrientation());
      tempTransform.setTranslation(anklePose.getPosition());
      tempTransform.multiplyInvertOther(transformFromAnkleToSole);
      footstepPose.setPoseIncludingFrame(anklePose.getReferenceFrame(), tempTransform);
   }

   public void addOffset(FrameVector3D offset)
   {
      footstepPose.prependTranslation(offset.getVector());

      for (int pointIdx = 0; pointIdx < customPositionWaypoints.size(); pointIdx++)
      {
         customPositionWaypoints.get(pointIdx).add(offset);
      }

      for (int pointIdx = 0; pointIdx < swingTrajectory.size(); pointIdx++)
      {
         FrameSE3TrajectoryPoint trajectoryPoint = swingTrajectory.get(pointIdx);
         trajectoryPoint.getPoseIncludingFrame(tempPose);
         tempPose.prependTranslation(offset.getVector());
         trajectoryPoint.setPosition(tempPose.getPosition());
      }
   }

}
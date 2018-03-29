package us.ihmc.commonWalkingControlModules.controlModules.flight;

import java.util.List;

import afu.org.checkerframework.checker.units.qual.m;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionNode;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionPlanner;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionPlannerParameters;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.ForceTrajectory;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.LinearControlModuleHelper;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

/**
 * Wrapper around the {@code CentroidalMotionPlanner} to enable high level objectives to be translated to 
 * motion planning inputs
 * @author Apoorv S
 *
 */
public class WholeBodyMotionPlanner
{
   public static final int numberOfForceCoefficients = LinearControlModuleHelper.forceCoefficients;
   public static final int maxNumberOfSegments = 20;

   private final ReferenceFrame planningFrame = ReferenceFrame.getWorldFrame();
   private final CentroidalMotionPlanner motionPlanner;
   private final CentroidalMotionNode motionNode;

   private final FramePoint3D initialPosition;
   private final FrameVector3D initialVelocity;
   private final FrameVector3D initialAccleration;

   private final FramePoint3D finalPosition;
   private final FrameVector3D finalVelocity;
   private final FrameVector3D finalAccleration;

   private final double gravityZ;
   private final double robotMass;
   private final double plannerDT;
   private final double minStandingHeight;
   private final double maxStandingHeight;

   private final FrameVector3D tempVector = new FrameVector3D();
   private final FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private final FrameVector3D nominalForce;
   private final FrameVector3D nominalForceRate;
   private final FramePoint3D positionLowerBound;
   private final FramePoint3D positionUpperBound;

   public WholeBodyMotionPlanner(CentroidalMotionPlannerParameters parameters)
   {
      this.plannerDT = parameters.getDeltaTMin();
      this.gravityZ = parameters.getGravityZ();
      this.robotMass = parameters.getRobotMass();
      this.maxStandingHeight = 1.0;
      this.minStandingHeight = 0.5;
      this.motionPlanner = new CentroidalMotionPlanner(parameters);
      this.motionNode = new CentroidalMotionNode(planningFrame);
      this.initialPosition = new FramePoint3D(planningFrame);
      this.initialVelocity = new FrameVector3D(planningFrame);
      this.initialAccleration = new FrameVector3D(planningFrame);
      this.finalPosition = new FramePoint3D(planningFrame);
      this.finalVelocity = new FrameVector3D(planningFrame);
      this.finalAccleration = new FrameVector3D(planningFrame);
      this.nominalForce = new FrameVector3D(planningFrame, 0.0, 0.0, -gravityZ * robotMass);
      this.nominalForceRate = new FrameVector3D(planningFrame);
      this.positionLowerBound = new FramePoint3D(planningFrame, Double.NaN, Double.NaN, minStandingHeight);
      this.positionUpperBound = new FramePoint3D(planningFrame, Double.NaN, Double.NaN, maxStandingHeight);
   }

   public void setInitialState(FramePoint3D positionToSet, FrameVector3D velocityToSet, FrameVector3D accelerationToSet)
   {
      this.initialPosition.setIncludingFrame(positionToSet);
      this.initialPosition.changeFrame(planningFrame);
      this.initialVelocity.setIncludingFrame(velocityToSet);
      this.initialVelocity.changeFrame(planningFrame);
      this.initialAccleration.setIncludingFrame(accelerationToSet);
      this.initialAccleration.changeFrame(planningFrame);
   }

   public void setFinalState(FramePoint3D positionToSet, FrameVector3D velocityToSet, FrameVector3D accelerationToSet)
   {
      this.finalPosition.setIncludingFrame(positionToSet);
      this.finalPosition.changeFrame(planningFrame);
      this.finalVelocity.setIncludingFrame(velocityToSet);
      this.finalVelocity.changeFrame(planningFrame);
      this.finalAccleration.setIncludingFrame(accelerationToSet);
      this.finalAccleration.changeFrame(planningFrame);
   }

   public void generateCoMPlanUsingContactStates(List<ContactState> contactStateList)
   {
      if(contactStateList.size() < 1)
         return;
      double nodeTime = 0.0;
      motionNode.reset();
      motionNode.setTime(nodeTime);
      motionNode.setPositionConstraint(initialPosition);
      motionNode.setLinearVelocityConstraint(initialVelocity);
      tempVector.setIncludingFrame(initialAccleration);
      tempVector.addZ(-gravityZ);
      tempVector.scale(robotMass);
      motionNode.setForceConstraint(tempVector);
      motionPlanner.submitNode(motionNode);
      for (int i = 0; i < contactStateList.size() - 1; i++)
      {
         ContactState contactState = contactStateList.get(i);
         ContactType contactType = contactState.getContactType();
         ContactType nextContactType = contactStateList.get(i + 1).getContactType();
         double contactStateDuration = contactState.getDuration();
         int numberOfNodesInState = (int) Math.ceil(contactStateDuration / plannerDT);
         for (int j = 0; j < numberOfNodesInState - 1; j++)
         {
            nodeTime += plannerDT;
            motionNode.reset();
            motionNode.setTime(nodeTime);
            switch (contactType)
            {
            case NO_SUPPORT:
               motionNode.setZeroForceConstraint();
               motionNode.setZeroForceRateConstraint();
               break;
            case DOUBLE_SUPPORT:
               motionNode.setForceObjective(nominalForce);
               motionNode.setForceRateObjective(nominalForceRate);
               motionNode.setPositionInequalities(positionUpperBound, positionLowerBound);
               break;
            default:
               throw new RuntimeException("Unknown support state. Cannot populate motion planner nodes");
            }
            motionPlanner.submitNode(motionNode);
         }
         double finalNodeDuration = contactStateDuration - (numberOfNodesInState - 1) * plannerDT;
         nodeTime += finalNodeDuration;
         motionNode.reset();
         motionNode.setTime(nodeTime);
         if (contactType == ContactType.NO_SUPPORT || nextContactType == ContactType.NO_SUPPORT)
         {
            motionNode.setZeroForceConstraint();
            motionNode.setZeroForceRateConstraint();
         }
         else
         {
            motionNode.setForceObjective(nominalForce);
            motionNode.setForceRateObjective(nominalForceRate);
            motionNode.setPositionInequalities(positionUpperBound, positionLowerBound);
         }
         motionPlanner.submitNode(motionNode);
      }
      ContactState contactState = contactStateList.get(contactStateList.size() - 1);
      ContactType contactType = contactState.getContactType();
      double contactStateDuration = contactState.getDuration();
      int numberOfNodesInState = (int) Math.ceil(contactStateDuration / plannerDT);
      for (int j = 0; j < numberOfNodesInState - 1; j++)
      {
         nodeTime += plannerDT;
         motionNode.reset();
         motionNode.setTime(nodeTime);
         switch (contactType)
         {
         case NO_SUPPORT:
            motionNode.setZeroForceConstraint();
            motionNode.setZeroForceRateConstraint();
            break;
         case DOUBLE_SUPPORT:
            motionNode.setForceObjective(nominalForce);
            motionNode.setForceRateObjective(nominalForceRate);
            motionNode.setPositionInequalities(positionUpperBound, positionLowerBound);
            break;
         default:
            throw new RuntimeException("Unknown support state. Cannot populate motion planner nodes");
         }
         motionPlanner.submitNode(motionNode);
      }
      double finalNodeDuration = contactStateDuration - (numberOfNodesInState - 1) * plannerDT;
      nodeTime += finalNodeDuration;
      motionNode.reset();
      motionNode.setTime(nodeTime);
      if (contactType == ContactType.NO_SUPPORT)
      {
         motionNode.setZeroForceConstraint();
         motionNode.setZeroForceRateConstraint();
      }
      else
      {
         motionNode.setForceObjective(nominalForce);
         motionNode.setForceRateObjective(nominalForceRate);
         motionNode.setPositionInequalities(positionUpperBound, positionLowerBound);
      }
      motionPlanner.submitNode(motionNode);
   }

   public ForceTrajectory getGroundReactionForceProfile()
   {
      return motionPlanner.getForceProfile();
   }

   public List<CentroidalMotionNode> getPlannerNodes()
   {
      return motionPlanner.getNodeList();
   }
}
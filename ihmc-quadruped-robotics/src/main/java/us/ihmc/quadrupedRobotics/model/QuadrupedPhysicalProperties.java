package us.ihmc.quadrupedRobotics.model;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedPhysicalProperties
{
   public double getNominalCoMHeight();
   public RigidBodyTransform getSoleToParentTransform(RobotQuadrant robotQuadrant);
}

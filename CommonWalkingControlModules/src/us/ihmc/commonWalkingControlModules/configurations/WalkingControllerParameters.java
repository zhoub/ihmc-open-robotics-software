package us.ihmc.commonWalkingControlModules.configurations;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.robotSide.SideDependentList;

public interface WalkingControllerParameters
{

   public abstract SideDependentList<Transform3D> getDesiredHandPosesWithRespectToChestFrame();
   
   public abstract Vector3d getDesiredHeadOffsetWithRespectToNeck();

   public abstract double getDesiredCoMHeight();

}
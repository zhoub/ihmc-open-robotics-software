package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameConnection implements ReferenceFrameHolder, EpsilonComparable<FrameConnection>
{
   private FrameConnectionPoint3D source;
   private FrameConnectionPoint3D target;

   public FrameConnection(FrameConnection other)
   {
      this.source = new FrameConnectionPoint3D(other.source);
      this.target = new FrameConnectionPoint3D(other.target);
   }

   public FrameConnection(FramePoint2DReadOnly source, int sourceRegionId, FramePoint2DReadOnly target, int targetRegionId)
   {
      target.checkReferenceFrameMatch(source);
      this.source = new FrameConnectionPoint3D(source, sourceRegionId);
      this.target = new FrameConnectionPoint3D(target, targetRegionId);
   }

   public FrameConnection(FrameConnectionPoint3D source, FrameConnectionPoint3D target)
   {
      target.checkReferenceFrameMatch(target);
      this.source = new FrameConnectionPoint3D(source);
      this.target = new FrameConnectionPoint3D(target);
   }

   public FrameConnection(FramePoint3DReadOnly source, int sourceRegionId, FramePoint3DReadOnly target, int targetRegionId)
   {
      target.checkReferenceFrameMatch(target);
      this.source = new FrameConnectionPoint3D(source, sourceRegionId);
      this.target = new FrameConnectionPoint3D(target, targetRegionId);
   }

   public FrameConnection getCopy()
   {
      return new FrameConnection(this);
   }

   public FrameConnectionPoint3D getSourcePoint()
   {
      return source;
   }

   public FrameConnectionPoint3D getTargetPoint()
   {
      return target;
   }

   public FramePoint2D getSourcePoint2D()
   {
      return new FramePoint2D(source);
   }

   public FramePoint2D getTargetPoint2D()
   {
      return new FramePoint2D(target);
   }

   public double distanceSquared(FramePoint3DReadOnly query)
   {
      checkReferenceFrames(query);
      return EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(query, source, target);
   }

   public double percentageAlongConnection(FramePoint3DReadOnly query)
   {
      checkReferenceFrames(query);
      return EuclidGeometryTools.percentageAlongLineSegment3D(query, source, target);
   }

   public FrameConnectionPoint3D orthogonalProjection(FramePoint3DReadOnly pointToProject, int regionId)
   {
      checkReferenceFrames(pointToProject);
      Point3D projection = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(pointToProject, source, target);
      return new FrameConnectionPoint3D(projection, regionId, pointToProject.getReferenceFrame());
   }

   public FrameConnectionPoint3D getPointGivenPercentage(double percentage, int regionId)
   {
      FramePoint3D result = new FramePoint3D();
      result.interpolate(source, target, percentage);
      return new FrameConnectionPoint3D(result, regionId);
   }

   public void flip()
   {
      FrameConnectionPoint3D temp = source;
      source = target;
      target = temp;
   }

   public FrameConnectionPoint3D getOppositePoint(FrameConnectionPoint3D point)
   {
      if (point.equals(source))
         return target;
      else if (point.equals(target))
         return source;
      return null;
   }

   public double length()
   {
      return source.distance(target);
   }

   public double lengthSquared()
   {
      return source.distanceSquared(target);
   }


   @Override
   public boolean epsilonEquals(FrameConnection other, double epsilon)
   {
      return source.epsilonEquals(other.source, epsilon) && target.epsilonEquals(other.target, epsilon) || source.epsilonEquals(other.target, epsilon) && target
            .epsilonEquals(other.source, epsilon);
   }

   @Override
   public int hashCode()
   {
      return source.hashCode() + target.hashCode();
   }

   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((FrameConnection) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(FrameConnection other)
   {
      if (other == null)
         return false;
      else
         return (source.equals(other.source) && target.equals(other.target)) || (source.equals(other.target) && target.equals(other.source));
   }

   @Override
   public String toString()
   {
      return "Connection: source = " + EuclidCoreIOTools.getTuple3DString(source) + ", target = " + EuclidCoreIOTools.getTuple3DString(target);
   }

   private void checkReferenceFrames(FrameTuple2DReadOnly query)
   {
      query.checkReferenceFrameMatch(source);
      query.checkReferenceFrameMatch(target);
   }

   private void checkReferenceFrames(FrameTuple3DReadOnly query)
   {
      query.checkReferenceFrameMatch(source);
      query.checkReferenceFrameMatch(target);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      target.checkReferenceFrameMatch(source);
      return source.getReferenceFrame();
   }
}
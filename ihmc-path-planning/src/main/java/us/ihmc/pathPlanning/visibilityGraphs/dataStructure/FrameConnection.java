package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;

public class FrameConnection implements ReferenceFrameHolder, FrameChangeable, EpsilonComparable<FrameConnection>
{
   private FrameConnectionPoint3D source;
   private FrameConnectionPoint3D target;

   private ReferenceFrame referenceFrame;

   public FrameConnection(FrameConnection other)
   {
      this.referenceFrame = other.getReferenceFrame();
      this.source = new FrameConnectionPoint3D(other.source);
      this.target = new FrameConnectionPoint3D(other.target);
   }

   public FrameConnection(FramePoint2DReadOnly source, int sourceRegionId, FramePoint2DReadOnly target, int targetRegionId)
   {
      source.checkReferenceFrameMatch(target);
      this.source = new FrameConnectionPoint3D(source, sourceRegionId);
      this.target = new FrameConnectionPoint3D(target, targetRegionId);
      referenceFrame = source.getReferenceFrame();
   }

   public FrameConnection(FrameConnectionPoint3DReadOnly source, FrameConnectionPoint3DReadOnly target)
   {
      source.checkReferenceFrameMatch(target);
      this.source = new FrameConnectionPoint3D(source);
      this.target = new FrameConnectionPoint3D(target);
      referenceFrame = source.getReferenceFrame();
   }

   public FrameConnection(FramePoint3DReadOnly source, int sourceRegionId, FramePoint3DReadOnly target, int targetRegionId)
   {
      source.checkReferenceFrameMatch(target);
      this.source = new FrameConnectionPoint3D(source, sourceRegionId);
      this.target = new FrameConnectionPoint3D(target, targetRegionId);
      referenceFrame = source.getReferenceFrame();
   }

   public FrameConnection getCopy()
   {
      return new FrameConnection(this);
   }

   public FrameConnectionPoint3DReadOnly getSourcePoint()
   {
      return source;
   }

   public FrameConnectionPoint3DReadOnly getTargetPoint()
   {
      return target;
   }

   public FramePoint2DReadOnly getSourcePoint2D()
   {
      return new FramePoint2D(source);
   }

   public FramePoint2DReadOnly getTargetPoint2D()
   {
      return new FramePoint2D(target);
   }

   public double distanceSquared(FramePoint3DReadOnly query)
   {
      query.checkReferenceFrameMatch(referenceFrame);
      return EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(query, source, target);
   }

   public double percentageAlongConnection(FramePoint3DReadOnly query)
   {
      query.checkReferenceFrameMatch(referenceFrame);
      return EuclidGeometryTools.percentageAlongLineSegment3D(query, source, target);
   }

   public FrameConnectionPoint3D orthogonalProjection(FramePoint3DReadOnly pointToProject, int regionId)
   {
      pointToProject.checkReferenceFrameMatch(referenceFrame);
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

   public FrameConnectionPoint3D getOppositePoint(FrameConnectionPoint3DReadOnly point)
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

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      target.setReferenceFrame(referenceFrame);
      source.setReferenceFrame(referenceFrame);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      target.applyTransform(transform);
      source.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      target.applyInverseTransform(transform);
      source.applyInverseTransform(transform);
   }
}
package us.ihmc.pathPlanning.visibilityGraphs.clusterManagement;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class FrameCluster implements FrameChangeable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private ReferenceFrame localFrame;

   private final List<FramePoint3DBasics> rawPoints = new ArrayList<>();
   private final List<FramePoint2DBasics> navigableExtrusions = new ArrayList<>();
   private final List<FramePoint2DBasics> nonNavigableExtrusions = new ArrayList<>();
   private List<? extends Point2DReadOnly> nonNavigableExtrusionsInLocal;

   private final BoundingBox2D nonNavigableExtrusionsBoundingBoxInLocal = new BoundingBox2D();

   private ExtrusionSide extrusionSide = ExtrusionSide.OUTSIDE;
   private Type type = Type.POLYGON;

   public FrameCluster(ReferenceFrame localFrame)
   {
      this.localFrame = localFrame;
   }

   public FrameCluster(int id, RigidBodyTransform transformToWorld)
   {
      localFrame = new ReferenceFrame("localClusterFrame" + id, worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformToWorld);
         }
      };
      localFrame.update();
   }

   public void updateBoundingBox()
   {
      nonNavigableExtrusions.forEach(extrusion -> {
         extrusion.changeFrame(localFrame);
         nonNavigableExtrusionsBoundingBoxInLocal.updateToIncludePoint(new Point2D(extrusion));
      });
   }

   public boolean isInsideNonNavigableZone(FramePoint2DReadOnly query)
   {
      query.checkReferenceFrameMatch(localFrame);
      if (nonNavigableExtrusionsInLocal == null)
      {
         nonNavigableExtrusionsInLocal = nonNavigableExtrusions.stream().map(extrusion -> {
            extrusion.changeFrame(localFrame);
            return new Point2D(extrusion);
         }).collect(Collectors.toList());
      }

      if (extrusionSide == ExtrusionSide.INSIDE)
      {
         if (!nonNavigableExtrusionsBoundingBoxInLocal.isInsideInclusive(query))
            return true;
         return !PlanarRegionTools.isPointInsidePolygon(nonNavigableExtrusionsInLocal, query);
      }
      else
      {
         if (!nonNavigableExtrusionsBoundingBoxInLocal.isInsideInclusive(query))
            return false;
         return PlanarRegionTools.isPointInsidePolygon(nonNavigableExtrusionsInLocal, query);
      }
   }

   public void setExtrusionSide(ExtrusionSide extrusionSide)
   {
      this.extrusionSide = extrusionSide;
   }

   public ExtrusionSide getExtrusionSide()
   {
      return extrusionSide;
   }

   public void setType(Type type)
   {
      this.type = type;
   }

   public Type getType()
   {
      return type;
   }

   public int getNumberOfRawPoints()
   {
      return rawPoints.size();
   }

   public void addRawPoint(FramePoint3DReadOnly point)
   {
      point.checkReferenceFrameMatch(localFrame);
      rawPoints.add(new FramePoint3D(point));
   }

   public void addRawPoints(List<? extends FramePoint3DReadOnly> points)
   {
      points.forEach(this::addRawPoint);
   }

   public void addRawPoint(ReferenceFrame referenceFrame, Point3DReadOnly point)
   {
      referenceFrame.checkReferenceFrameMatch(localFrame);
      rawPoints.add(new FramePoint3D(referenceFrame, point));
   }

   public void addRawPoint(ReferenceFrame referenceFrame, Point2DReadOnly point)
   {
      referenceFrame.checkReferenceFrameMatch(localFrame);
      rawPoints.add(new FramePoint3D(referenceFrame, point));
   }

   public void addRawPoints(ReferenceFrame referenceFrame, Collection<? extends Point3DReadOnly> points)
   {
      points.forEach(point -> addRawPoint(referenceFrame, point));
   }

   public void addRawPoints(ReferenceFrame referenceFrame, Point2DReadOnly[] points)
   {
      for (Point2DReadOnly point : points)
         addRawPoint(referenceFrame, point);
   }

   public FramePoint3DReadOnly getRawPoint(int i)
   {
      return rawPoints.get(i);
   }

   public List<? extends FramePoint3DReadOnly> getRawPoints()
   {
      return rawPoints;
   }

   public void addNavigableExtrusionInLocal(Point2DReadOnly navigableExtrusionInLocal)
   {
      navigableExtrusions.add(new FramePoint2D(localFrame, navigableExtrusionInLocal));
   }

   public void addNavigableExtrusionInLocal(Point3DReadOnly navigableExtrusionInLocal)
   {
      navigableExtrusions.add(new FramePoint2D(localFrame, navigableExtrusionInLocal));
   }

   public void addNavigableExtrusionsInLocal(List<? extends Point3DReadOnly> navigableExtrusionInLocal)
   {
      navigableExtrusionInLocal.forEach(this::addNavigableExtrusionInLocal);
   }

   public void addNavigableExtrusion(FramePoint2DBasics navigableExtrusion)
   {
      navigableExtrusion.checkReferenceFrameMatch(localFrame);
      navigableExtrusions.add(navigableExtrusion);
   }

   public void addNavigableExtrusions(List<FramePoint2DBasics> navigableExtrusion)
   {
      navigableExtrusion.forEach(this::addNavigableExtrusionInLocal);
   }

   public int getNumberOfNavigableExtrusions()
   {
      return navigableExtrusions.size();
   }

   public FramePoint2DReadOnly getNavigableExtrusion(int i)
   {
      return navigableExtrusions.get(i);
   }

   public List<FramePoint2DBasics> getNavigableExtrusions()
   {
      return navigableExtrusions;
   }

   public void addNonNavigableExtrusionInLocal(Point2DReadOnly nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionsInLocal = null;
      nonNavigableExtrusions.add(new FramePoint2D(localFrame, nonNavigableExtrusionInLocal));
   }

   public void addNonNavigableExtrusionInLocal(Point3DReadOnly nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionsInLocal = null;
      nonNavigableExtrusions.add(new FramePoint2D(localFrame, nonNavigableExtrusionInLocal));
   }

   public void addNonNavigableExtrusionsInLocal(List<? extends Point3DReadOnly> nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionInLocal.forEach(this::addNonNavigableExtrusionInLocal);
   }

   public void addNonNavigableExtrusion(FramePoint2DBasics nonNavigableExtrusion)
   {
      nonNavigableExtrusionsInLocal = null;
      nonNavigableExtrusion.checkReferenceFrameMatch(localFrame);
      nonNavigableExtrusions.add(nonNavigableExtrusion);
   }

   public void addNonNavigableExtrusions(List<FramePoint2DBasics> nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionInLocal.forEach(this::addNonNavigableExtrusion);
   }

   public BoundingBox2D getNonNavigableExtrusionsBoundingBox()
   {
      return nonNavigableExtrusionsBoundingBoxInLocal;
   }

   public int getNumberOfNonNavigableExtrusions()
   {
      return nonNavigableExtrusions.size();
   }

   public FramePoint2DReadOnly getNonNavigableExtrusion(int i)
   {
      return nonNavigableExtrusions.get(i);
   }

   public List<? extends FramePoint2DReadOnly> getNonNavigableExtrusions()
   {
      return nonNavigableExtrusions;
   }


   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.localFrame = referenceFrame;
      for (FramePoint3DBasics rawPoint : rawPoints)
         rawPoint.setReferenceFrame(referenceFrame);
      for (FramePoint2DBasics navigableExtrusion : navigableExtrusions)
         navigableExtrusion.setReferenceFrame(referenceFrame);
      for (FramePoint2DBasics nonNavigableExtrusion : nonNavigableExtrusions)
         nonNavigableExtrusion.setReferenceFrame(referenceFrame);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      for (FramePoint3DBasics rawPoint : rawPoints)
         rawPoint.applyTransform(transform);
      for (FramePoint2DBasics navigableExtrusion : navigableExtrusions)
         navigableExtrusion.applyTransform(transform);
      for (FramePoint2DBasics nonNavigableExtrusion : nonNavigableExtrusions)
         nonNavigableExtrusion.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (FramePoint3DBasics rawPoint : rawPoints)
         rawPoint.applyInverseTransform(transform);
      for (FramePoint2DBasics navigableExtrusion : navigableExtrusions)
         navigableExtrusion.applyInverseTransform(transform);
      for (FramePoint2DBasics nonNavigableExtrusion : nonNavigableExtrusions)
         nonNavigableExtrusion.applyInverseTransform(transform);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return localFrame;
   }
}

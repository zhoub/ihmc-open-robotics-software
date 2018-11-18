package us.ihmc.pathPlanning.visibilityGraphs.clusterManagement;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class FrameCluster
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame localFrame = new ReferenceFrame("localClusterFrame", worldFrame)
   {
      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.set(transformToWorld);
      }
   };

   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private final List<FramePoint3DReadOnly> rawPoints = new ArrayList<>();
   private final List<FramePoint3DReadOnly> navigableExtrusions = new ArrayList<>();
   private final List<FramePoint3DBasics> nonNavigableExtrusions = new ArrayList<>();
   private List<? extends Point2DReadOnly> nonNavigableExtrusionsInLocal;

   private final BoundingBox2D nonNavigableExtrusionsBoundingBoxInLocal = new BoundingBox2D();

   public enum ExtrusionSide
   {
      INSIDE, OUTSIDE;

      public static ExtrusionSide[] values = values();

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static ExtrusionSide fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   }

   ;

   private ExtrusionSide extrusionSide = ExtrusionSide.OUTSIDE;

   public enum Type
   {
      LINE, MULTI_LINE, POLYGON;

      public static Type[] values = values();

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static Type fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   }

   ;

   private Type type = Type.POLYGON;

   public FrameCluster()
   {
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

   public void setTransformToWorld(RigidBodyTransform transform)
   {
      transformToWorld.set(transform);
      localFrame.update();
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
   }

   public int getNumberOfRawPoints()
   {
      return rawPoints.size();
   }

   public void addRawPoint(FramePoint3DReadOnly point)
   {
      rawPoints.add(new FramePoint3D(point));
   }

   public void addRawPointInLocal(Point2DReadOnly pointInLocal)
   {
      rawPoints.add(new FramePoint3D(localFrame, pointInLocal));
   }

   public void addRawPointInLocal(Point3DReadOnly pointInLocal)
   {
      rawPoints.add(new FramePoint3D(localFrame, pointInLocal));
   }

   public void addRawPointInWorld(Point3DReadOnly pointInWorld)
   {
      rawPoints.add(new FramePoint3D(worldFrame, pointInWorld));
   }

   public void addRawPointsInLocal2D(List<? extends Point2DReadOnly> pointsInLocal)
   {
      pointsInLocal.forEach(this::addRawPointInLocal);
   }

   public void addRawPointsInLocal3D(List<? extends Point3DReadOnly> pointsInLocal)
   {
      pointsInLocal.forEach(this::addRawPointInLocal);
   }

   public void addRawPointsInWorld(List<? extends Point3DReadOnly> pointsInWorld)
   {
      pointsInWorld.forEach(this::addRawPointInWorld);
   }

   public void addRawPointsInLocal2D(Point2DReadOnly[] pointsInLocal)
   {
      addRawPointsInLocal2D(Arrays.asList(pointsInLocal));
   }

   public FramePoint3DReadOnly getRawPoint(int i)
   {
      return rawPoints.get(i);
   }

   public List<FramePoint3DReadOnly> getRawPoints()
   {
      return rawPoints;
   }

   public void addNavigableExtrusionInLocal(Point2DReadOnly navigableExtrusionInLocal)
   {
      navigableExtrusions.add(new FramePoint3D(localFrame, navigableExtrusionInLocal));
   }

   public void addNavigableExtrusionInLocal(Point3DReadOnly navigableExtrusionInLocal)
   {
      navigableExtrusions.add(new FramePoint3D(localFrame, navigableExtrusionInLocal));
   }

   public void addNavigableExtrusionsInLocal(List<? extends Point3DReadOnly> navigableExtrusionInLocal)
   {
      navigableExtrusionInLocal.forEach(this::addNavigableExtrusionInLocal);
   }

   public int getNumberOfNavigableExtrusions()
   {
      return navigableExtrusions.size();
   }

   public FramePoint3DReadOnly getNavigableExtrusion(int i)
   {
      return navigableExtrusions.get(i);
   }

   public List<FramePoint3DReadOnly> getNavigableExtrusions()
   {
      return navigableExtrusions;
   }

   public void addNonNavigableExtrusionInLocal(Point2DReadOnly nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionsInLocal = null;
      nonNavigableExtrusions.add(new FramePoint3D(localFrame, nonNavigableExtrusionInLocal));
   }

   public void addNonNavigableExtrusionInLocal(Point3DReadOnly nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionsInLocal = null;
      nonNavigableExtrusions.add(new FramePoint3D(localFrame, nonNavigableExtrusionInLocal));
   }

   public void addNonNavigableExtrusionsInLocal(List<? extends Point3DReadOnly> nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionInLocal.forEach(this::addNonNavigableExtrusionInLocal);
   }

   public BoundingBox2D getNonNavigableExtrusionsBoundingBox()
   {
      return nonNavigableExtrusionsBoundingBoxInLocal;
   }

   public int getNumberOfNonNavigableExtrusions()
   {
      return nonNavigableExtrusions.size();
   }

   public FramePoint3DReadOnly getNonNavigableExtrusion(int i)
   {
      return nonNavigableExtrusions.get(i);
   }

   public List<? extends FramePoint3DReadOnly> getNonNavigableExtrusions()
   {
      return nonNavigableExtrusions;
   }
}

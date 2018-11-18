package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class NavigableRegion implements VisibilityMapHolder
{
   private final double costOfRotating = 2.0;
   private final PlanarRegion homeRegion;
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private Cluster homeRegionCluster = null;
   private List<Cluster> obstacleClusters = new ArrayList<>();
   private List<Cluster> rotationClusters = new ArrayList<>();
   private List<Cluster> allClusters = new ArrayList<>();
   private VisibilityMap visibilityMapInLocal = null;
   private VisibilityMap visibilityMapInWorld = null;

   public NavigableRegion(PlanarRegion homeRegion)
   {
      this.homeRegion = homeRegion;
      homeRegion.getTransformToWorld(transformToWorld);
   }

   public void setHomeRegionCluster(Cluster homeCluster)
   {
      this.homeRegionCluster = homeCluster;
      allClusters.add(homeCluster);
   }

   public void addObstacleClusters(Iterable<Cluster> obstacleClusters)
   {
      obstacleClusters.forEach(this::addObstacleCluster);
   }

   public void addObstacleCluster(Cluster obstacleCluster)
   {
      obstacleClusters.add(obstacleCluster);
      allClusters.add(obstacleCluster);
   }

   public void addRotationClusters(Iterable<Cluster> rotationClusters)
   {
      rotationClusters.forEach(this::addRotationCluster);
   }

   public void addRotationCluster(Cluster rotationCluster)
   {
      rotationClusters.add(rotationCluster);
   }

   public void setVisibilityMapInLocal(VisibilityMap visibilityMap)
   {
      visibilityMapInLocal = visibilityMap;
   }

   public void setVisibilityMapInWorld(VisibilityMap visibilityMap)
   {
      if (visibilityMapInLocal == null)
         visibilityMapInLocal = new VisibilityMap();

      visibilityMapInLocal.copy(visibilityMap);
      visibilityMapInLocal.applyInverseTransform(transformToWorld);
   }

   public PlanarRegion getHomeRegion()
   {
      return homeRegion;
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return new RigidBodyTransform(transformToWorld);
   }

   public Cluster getHomeRegionCluster()
   {
      return homeRegionCluster;
   }

   public List<Cluster> getObstacleClusters()
   {
      return obstacleClusters;
   }

   public List<Cluster> getRotationClusters()
   {
      return rotationClusters;
   }

   public List<Cluster> getAllClusters()
   {
      return allClusters;
   }


   public void transformFromWorldToLocal(Transformable objectToTransformToWorld)
   {
      objectToTransformToWorld.applyInverseTransform(transformToWorld);
   }

   @Override
   public int getMapId()
   {
      return homeRegion.getRegionId();
   }

   @Override
   public VisibilityMap getVisibilityMapInLocal()
   {
      return visibilityMapInLocal;
   }

   @Override
   public VisibilityMap getVisibilityMapInWorld()
   {
      if (visibilityMapInWorld == null)
      {
         visibilityMapInWorld = new VisibilityMap();
         visibilityMapInWorld.copy(visibilityMapInLocal);
         visibilityMapInWorld.applyTransform(transformToWorld);
      }

      return visibilityMapInWorld;
   }

   private final double maxDistanceInside = 0.3;
   @Override
   public double getConnectionWeight(Connection connectionInWorld)
   {
      Point2DReadOnly source = connectionInWorld.getSourcePoint2D();
      Point2DReadOnly target = connectionInWorld.getTargetPoint2D();
      if (source.equals(target))
         return connectionInWorld.length();

      for (Cluster cluster : rotationClusters)
      {
         double distanceInside = PlanarRegionTools.getDistanceOfLine2DInsideConvexPolygon(source, target, cluster.getRotationConvexRegionInWorld().getVertexBufferView());
         if (distanceInside > 0.0)
         {
            double fractionToMaxRotation = distanceInside / maxDistanceInside;
            return costOfRotating * (1.0 + Math.pow(fractionToMaxRotation, 2.0)) * connectionInWorld.length();
         }
      }

      return connectionInWorld.length();
   }

   public double getConnectionWeight(Point3DReadOnly source, Point3DReadOnly target)
   {
      return getConnectionWeight(new Point2D(source), new Point2D(target));
   }

   public double getConnectionWeight(Point2DReadOnly source, Point2DReadOnly target)
   {
      Connection connection = new Connection(new ConnectionPoint3D(source, 0), new ConnectionPoint3D(target, 0));
      return getConnectionWeight(connection);
   }
}
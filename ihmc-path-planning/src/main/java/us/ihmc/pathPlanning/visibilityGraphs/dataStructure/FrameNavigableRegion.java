package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.FrameCluster;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.FrameVisibilityMapHolder;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.List;

public class FrameNavigableRegion implements FrameChangeable, FrameVisibilityMapHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final PlanarRegion homeRegion;
   private ReferenceFrame localFrame;

   private FrameCluster homeRegionCluster = null;
   private List<FrameCluster> obstacleClusters = new ArrayList<>();
   private List<FrameCluster> allClusters = new ArrayList<>();
   private FrameVisibilityMap visibilityMapInLocal = null;

   public FrameNavigableRegion(PlanarRegion homeRegion)
   {
      this.homeRegion = homeRegion;

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      homeRegion.getTransformToWorld(transformToWorld);
      localFrame = new ReferenceFrame("localFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformToWorld);
         }
      };
      localFrame.update();
   }

   public void setHomeRegionCluster(FrameCluster homeCluster)
   {
      this.homeRegionCluster = homeCluster;
      allClusters.add(homeCluster);
   }

   public void addObstacleClusters(Iterable<FrameCluster> obstacleClusters)
   {
      obstacleClusters.forEach(this::addObstacleCluster);
   }

   public void addObstacleCluster(FrameCluster obstacleCluster)
   {
      obstacleClusters.add(obstacleCluster);
      allClusters.add(obstacleCluster);
   }

   public void setVisibilityMap(FrameVisibilityMap visibilityMap)
   {
      visibilityMapInLocal = visibilityMap;
   }

   public PlanarRegion getHomeRegion()
   {
      return homeRegion;
   }

   public FrameCluster getHomeRegionCluster()
   {
      return homeRegionCluster;
   }

   public List<FrameCluster> getObstacleClusters()
   {
      return obstacleClusters;
   }

   public List<FrameCluster> getAllClusters()
   {
      return allClusters;
   }

   @Override
   public int getMapId()
   {
      return homeRegion.getRegionId();
   }

   @Override
   public FrameVisibilityMap getVisibilityMap()
   {
      return visibilityMapInLocal;
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      localFrame = referenceFrame;
      if (homeRegionCluster != null)
         homeRegionCluster.setReferenceFrame(referenceFrame);
      if (visibilityMapInLocal != null)
         visibilityMapInLocal.setReferenceFrame(referenceFrame);
      for (FrameCluster cluster : allClusters)
         cluster.setReferenceFrame(referenceFrame);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      if (homeRegionCluster != null)
         homeRegionCluster.applyTransform(transform);
      if (visibilityMapInLocal != null)
         visibilityMapInLocal.applyTransform(transform);
      for (FrameCluster cluster : allClusters)
         cluster.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      if (homeRegionCluster != null)
         homeRegionCluster.applyInverseTransform(transform);
      if (visibilityMapInLocal != null)
         visibilityMapInLocal.applyInverseTransform(transform);
      for (FrameCluster cluster : allClusters)
         cluster.applyInverseTransform(transform);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return localFrame;
   }
}
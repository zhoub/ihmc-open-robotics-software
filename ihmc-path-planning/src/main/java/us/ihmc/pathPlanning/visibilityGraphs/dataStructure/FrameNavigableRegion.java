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

public class FrameNavigableRegion implements FrameChangeable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final PlanarRegion homeRegion;
   private final ReferenceFrame localFrame = new ReferenceFrame("localFrame", worldFrame)
   {
      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.set(transformToWorld);
      }
   };
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private FrameCluster homeRegionCluster = null;
   private List<FrameCluster> obstacleClusters = new ArrayList<>();
   private List<FrameCluster> allClusters = new ArrayList<>();
   private FrameVisibilityMap visibilityMapInLocal = null;

   public FrameNavigableRegion(PlanarRegion homeRegion)
   {
      this.homeRegion = homeRegion;
      homeRegion.getTransformToWorld(transformToWorld);
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

}
package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.FrameCluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.FrameNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;

public class OcclusionTools
{
   public static boolean isTheGoalIntersectingAnyObstacles(NavigableRegion region, Point3DReadOnly start, Point3DReadOnly goal)
   {
      for (Cluster cluster : region.getObstacleClusters())
      {
         ArrayList<Point2D> list2D = new ArrayList<>();

         for (Point3DReadOnly point3d : cluster.getNonNavigableExtrusionsInWorld())
         {
            list2D.add(new Point2D(point3d));
         }

         boolean visible = VisibilityTools.isPointVisible(new Point2D(start), new Point2D(goal), list2D);

         if (!visible)
         {
            return true;
         }
      }

      return false;
   }

   public static List<Cluster> getListOfIntersectingObstacles(List<Cluster> obstacleClusters, Point3DReadOnly start, Point3DReadOnly goal)
   {
      List<Cluster> clustersTemp = new ArrayList<Cluster>();
      for (Cluster cluster : obstacleClusters)
      {
         ArrayList<Point2D> list2D = new ArrayList<>();

         for (Point3DReadOnly point3d : cluster.getNonNavigableExtrusionsInWorld())
         {
            list2D.add(new Point2D(point3d));
         }

         boolean visible = VisibilityTools.isPointVisible(new Point2D(start), new Point2D(goal), list2D);

         if (!visible)
         {
            clustersTemp.add(cluster);
         }
      }
      return clustersTemp;
   }

   public static boolean isTheGoalIntersectingAnyObstacles(FrameNavigableRegion region, FramePoint3DReadOnly start, FramePoint3DReadOnly goal)
   {
      for (FrameCluster cluster : region.getObstacleClusters())
      {
         ArrayList<FramePoint2D> list2D = new ArrayList<>();

         for (FramePoint2DReadOnly point3d : cluster.getNonNavigableExtrusions())
         {
            list2D.add(new FramePoint2D(point3d));
         }

         boolean visible = VisibilityTools.isPointVisible(new FramePoint2D(start), new FramePoint2D(goal), list2D);

         if (!visible)
         {
            return true;
         }
      }

      return false;
   }

   public static List<FrameCluster> getListOfIntersectingObstacles(List<FrameCluster> obstacleClusters, FramePoint3DReadOnly start, FramePoint3DReadOnly goal)
   {
      List<FrameCluster> clustersTemp = new ArrayList<>();
      for (FrameCluster cluster : obstacleClusters)
      {
         ArrayList<FramePoint2D> list2D = new ArrayList<>();

         for (FramePoint2DReadOnly point3d : cluster.getNonNavigableExtrusions())
         {
            list2D.add(new FramePoint2D(point3d));
         }

         boolean visible = VisibilityTools.isPointVisible(new FramePoint2D(start), new FramePoint2D(goal), list2D);

         if (!visible)
         {
            clustersTemp.add(cluster);
         }
      }
      return clustersTemp;
   }
}

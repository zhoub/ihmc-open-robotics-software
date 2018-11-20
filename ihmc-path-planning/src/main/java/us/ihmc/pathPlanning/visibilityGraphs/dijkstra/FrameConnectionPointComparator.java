package us.ihmc.pathPlanning.visibilityGraphs.dijkstra;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.FrameConnectionPoint3DReadOnly;

import java.util.Comparator;
import java.util.HashMap;

class FrameConnectionPointComparator implements Comparator<FrameConnectionPoint3DReadOnly>
{
   private final HashMap<FrameConnectionPoint3DReadOnly, Double> nodeCosts;

   FrameConnectionPointComparator(HashMap<FrameConnectionPoint3DReadOnly, Double> nodeCosts)
   {
      this.nodeCosts = nodeCosts;
   }

   @Override
   public int compare(FrameConnectionPoint3DReadOnly point1, FrameConnectionPoint3DReadOnly point2)
   {
      double cost1 = nodeCosts.get(point1);
      double cost2 = nodeCosts.get(point2);
      if(cost1 == cost2) return 0;
      return cost1 < cost2 ? -1 : 1;
   }
}

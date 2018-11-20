package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.FrameVisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

public class FrameInterRegionVisibilityMap implements FrameVisibilityMapHolder
{
   /**
    * This scale factor is a multiplier on inter-region connection weight. If this value greater
    * than 1.0, it will encourage paths with short inter-region connections. The benefit is discouraging
    * the use of long inter-region connections, which the planner favors when this scale is 1.0
    */
   private static final double INTER_REGION_WEIGHT_SCALE = 1.25;

   private final FrameVisibilityMap visibilityMap = new FrameVisibilityMap(ReferenceFrame.getWorldFrame());

   public void addConnections(Iterable<FrameConnection> connections)
   {
      connections.forEach(this::addConnection);
   }

   public void addConnection(FrameConnection connection)
   {
      visibilityMap.addConnection(connection);
   }

   public void addConnection(FrameConnectionPoint3DReadOnly source, FrameConnectionPoint3DReadOnly target)
   {
      visibilityMap.addConnection(new FrameConnection(source, target));
   }

   @Override
   public double getConnectionWeight(FrameConnection connection)
   {
      FrameConnectionPoint3DReadOnly sourcePoint = connection.getSourcePoint();
      FrameConnectionPoint3DReadOnly targetPoint = connection.getTargetPoint();

      double horizontalDistance = EuclidGeometryTools.distanceBetweenPoint2Ds(sourcePoint.getX(), sourcePoint.getY(), targetPoint.getX(), targetPoint.getY());
      double verticalDistance = Math.abs(sourcePoint.getZ() - targetPoint.getZ());

      return INTER_REGION_WEIGHT_SCALE * (horizontalDistance + verticalDistance);
   }

   @Override
   public int getMapId()
   {
      return 0;
   }

   @Override
   public FrameVisibilityMap getVisibilityMap()
   {
      return visibilityMap;
   }
}

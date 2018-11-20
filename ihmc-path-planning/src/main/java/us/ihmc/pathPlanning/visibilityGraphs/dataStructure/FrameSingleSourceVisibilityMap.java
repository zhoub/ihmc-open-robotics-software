package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.FrameVisibilityMapHolder;

import java.util.Collection;

public class FrameSingleSourceVisibilityMap implements FrameVisibilityMapHolder
{
   private final FramePoint3D source;
   private final int mapId;
   private final ReferenceFrame localFrame;
   private final FrameNavigableRegion hostRegion;
   private final FrameVisibilityMap visibilityMap;

   public FrameSingleSourceVisibilityMap(FramePoint3DReadOnly source, FrameNavigableRegion hostRegion, Collection<FrameConnection> connections)
   {
      this(source, hostRegion.getMapId(), hostRegion.getReferenceFrame(), connections);
   }

   public FrameSingleSourceVisibilityMap(FramePoint3DReadOnly source, int mapId, ReferenceFrame referenceFrame,
                                         Collection<FrameConnection> connections)
   {
      this.source = new FramePoint3D(source);
      this.hostRegion = null;
      this.localFrame = referenceFrame;
      this.mapId = mapId;
      this.source.changeFrame(localFrame);

      visibilityMap = new FrameVisibilityMap(localFrame, connections);
   }

   public void addConnection(FrameConnection connectionInWorld)
   {
      visibilityMap.addConnection(connectionInWorld);
      visibilityMap.computeVertices();
   }


   public FramePoint3DReadOnly getSource()
   {
      return source;
   }

   public FramePoint2DReadOnly getSource2D()
   {
      return new FramePoint2D(source);
   }

   public FrameNavigableRegion getHostRegion()
   {
      return hostRegion;
   }

   @Override
   public int getMapId()
   {
      return mapId;
   }

   @Override
   public FrameVisibilityMap getVisibilityMap()
   {
      return visibilityMap;
   }
}

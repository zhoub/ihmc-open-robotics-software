package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.FrameConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.FrameConnectionPoint3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.Collection;
import java.util.List;

public interface VisibilityGraphPathPlanner
{
   List<Point3DReadOnly> calculatePath(ConnectionPoint3D start, ConnectionPoint3D goal, Collection<VisibilityMapHolder> visibilityMapHolders);
   List<FramePoint3DReadOnly> calculatePath(FrameConnectionPoint3DReadOnly start, FrameConnectionPoint3DReadOnly goal,
                                            Collection<FrameVisibilityMapHolder> visibilityMapHolders);
}

package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.FrameVisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;

public interface FrameVisibilityMapHolder
{
   int getMapId();

   FrameVisibilityMap getVisibilityMap();

   default double getConnectionWeight(Connection connection)
   {
      return connection.length();
   }
}

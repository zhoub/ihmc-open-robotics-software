package us.ihmc.pathPlanning.visibilityGraphs.dijkstra;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.FrameConnection;

class FrameConnectionData
{
   FrameConnection connection;
   double edgeWeight;

   @Override
   public int hashCode()
   {
      return connection.hashCode();
   }
}

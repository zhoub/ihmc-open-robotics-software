package us.ihmc.pathPlanning.visibilityGraphs.dijkstra;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.FrameVisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.*;

public class DijkstraVisibilityGraphPlanner implements VisibilityGraphPathPlanner
{
   private final HashMap<ConnectionPoint3D, HashSet<ConnectionData>> visibilityMap = new HashMap<>();
   private final HashMap<ConnectionPoint3D, Double> nodeCosts = new HashMap<>();
   private final HashMap<ConnectionPoint3D, ConnectionData> incomingBestEdge = new HashMap<>();

   private PriorityQueue<ConnectionPoint3D> stack;

   private final HashMap<FrameConnectionPoint3DReadOnly, HashSet<FrameConnectionData>> frameVisibilityMap = new HashMap<>();
   private final HashMap<FrameConnectionPoint3DReadOnly, Double> frameNodeCosts = new HashMap<>();
   private final HashMap<FrameConnectionPoint3DReadOnly, FrameConnectionData> frameIncomingBestEdge = new HashMap<>();

   private PriorityQueue<FrameConnectionPoint3DReadOnly> frameStack;

   private void initialize(ConnectionPoint3D startPoint)
   {
      nodeCosts.put(startPoint, 0.0);
      stack = new PriorityQueue<>(new ConnectionPointComparator(nodeCosts));
      stack.add(startPoint);
   }

   private void buildVisibilityMap(Collection<VisibilityMapHolder> visibilityMapHolders)
   {
      visibilityMap.clear();
      nodeCosts.clear();

      for (VisibilityMapHolder visibilityMapHolder : visibilityMapHolders)
      {
         VisibilityMap visibilityMap = visibilityMapHolder.getVisibilityMapInWorld();
         for (Connection connection : visibilityMap)
         {
            ConnectionData connectionData = new ConnectionData();
            connectionData.connection = connection;
            connectionData.edgeWeight = visibilityMapHolder.getConnectionWeight(connection);

            this.visibilityMap.computeIfAbsent(connection.getSourcePoint(), (p) -> new HashSet<>()).add(connectionData);
            this.visibilityMap.computeIfAbsent(connection.getTargetPoint(), (p) -> new HashSet<>()).add(connectionData);
         }
      }
   }

   /**
    * Plans a path over the visibility maps
    * @return shortest path to the goal if one exists. Otherwise it will return a path to the
    * closest possible node to the goal
    */
   public List<Point3DReadOnly> calculatePath(ConnectionPoint3D startPoint, ConnectionPoint3D goalPoint, Collection<VisibilityMapHolder> visibilityMapHolders)
   {
      buildVisibilityMap(visibilityMapHolders);
      initialize(startPoint);

      ConnectionPoint3D closestPointToGoal = startPoint;
      double closestDistanceToGoalSquared = startPoint.distanceSquared(goalPoint);

      stackLoop:
      while(!stack.isEmpty())
      {
         ConnectionPoint3D sourcePoint = stack.poll();

         HashSet<ConnectionData> connections = visibilityMap.computeIfAbsent(sourcePoint, (p) -> new HashSet<>());
         double distanceToGoalSquared = sourcePoint.distanceSquared(goalPoint);
         if(distanceToGoalSquared < closestDistanceToGoalSquared)
         {
            closestPointToGoal = sourcePoint;
            closestDistanceToGoalSquared = distanceToGoalSquared;
         }

         for (ConnectionData connectionData : connections)
         {
            Connection connection = connectionData.connection;
            ConnectionPoint3D targetPoint = connection.getOppositePoint(sourcePoint);

            double nodeCost = nodeCosts.get(sourcePoint) + connectionData.edgeWeight;

            if(!nodeCosts.containsKey(targetPoint) || nodeCosts.get(targetPoint) > nodeCost)
            {
               nodeCosts.put(targetPoint, nodeCost);
               incomingBestEdge.put(targetPoint, connectionData);

               if(targetPoint.equals(goalPoint))
               {
                  break stackLoop;
               }
               else
               {
                  stack.add(targetPoint);
               }
            }
         }
      }

      if(nodeCosts.containsKey(goalPoint))
      {
         return getPathToPoint(goalPoint);
      }
      else
      {
         return getPathToPoint(closestPointToGoal);
      }
   }

   private List<Point3DReadOnly> getPathToPoint(ConnectionPoint3D point)
   {
      List<Point3DReadOnly> path = new ArrayList<Point3DReadOnly>(){{add(point);}};

      ConnectionData incomingEdge = incomingBestEdge.get(point);
      ConnectionPoint3D previousTargetPoint = new ConnectionPoint3D(point);

      while(incomingEdge != null)
      {
         Connection connection = incomingEdge.connection;
         ConnectionPoint3D targetPoint = connection.getOppositePoint(previousTargetPoint);
         path.add(targetPoint);
         incomingEdge = incomingBestEdge.get(targetPoint);

         previousTargetPoint = targetPoint;
      }

      Collections.reverse(path);
      return path;
   }

   private void initialize(FrameConnectionPoint3DReadOnly startPoint)
   {
      frameNodeCosts.put(startPoint, 0.0);
      frameStack = new PriorityQueue<>(new FrameConnectionPointComparator(frameNodeCosts));
      frameStack.add(startPoint);
   }


   private void buildFrameVisibilityMap(Collection<FrameVisibilityMapHolder> visibilityMapHolders)
   {
      frameVisibilityMap.clear();
      frameNodeCosts.clear();

      for (FrameVisibilityMapHolder visibilityMapHolder : visibilityMapHolders)
      {
         FrameVisibilityMap visibilityMap = visibilityMapHolder.getVisibilityMap();
         for (FrameConnection connection : visibilityMap)
         {
            FrameConnectionData connectionData = new FrameConnectionData();
            connectionData.connection = connection;
            connectionData.edgeWeight = visibilityMapHolder.getConnectionWeight(connection);

            this.frameVisibilityMap.computeIfAbsent(connection.getSourcePoint(), (p) -> new HashSet<>()).add(connectionData);
            this.frameVisibilityMap.computeIfAbsent(connection.getTargetPoint(), (p) -> new HashSet<>()).add(connectionData);
         }
      }
   }

   public List<FramePoint3DReadOnly> calculatePath(FrameConnectionPoint3DReadOnly startPoint, FrameConnectionPoint3DReadOnly goalPoint,
                                                   Collection<FrameVisibilityMapHolder> visibilityMapHolders)
   {
      buildFrameVisibilityMap(visibilityMapHolders);
      initialize(startPoint);

      FrameConnectionPoint3DReadOnly closestPointToGoal = startPoint;
      double closestDistanceToGoalSquared = startPoint.distanceSquared(goalPoint);

      stackLoop:
      while(!frameStack.isEmpty())
      {
         FrameConnectionPoint3DReadOnly sourcePoint = frameStack.poll();

         HashSet<FrameConnectionData> connections = frameVisibilityMap.computeIfAbsent(sourcePoint, (p) -> new HashSet<>());
         double distanceToGoalSquared = sourcePoint.distanceSquared(goalPoint);
         if(distanceToGoalSquared < closestDistanceToGoalSquared)
         {
            closestPointToGoal = sourcePoint;
            closestDistanceToGoalSquared = distanceToGoalSquared;
         }

         for (FrameConnectionData connectionData : connections)
         {
            FrameConnection connection = connectionData.connection;
            FrameConnectionPoint3DReadOnly targetPoint = connection.getOppositePoint(sourcePoint);

            double nodeCost = frameNodeCosts.get(sourcePoint) + connectionData.edgeWeight;

            if(!frameNodeCosts.containsKey(targetPoint) || frameNodeCosts.get(targetPoint) > nodeCost)
            {
               frameNodeCosts.put(targetPoint, nodeCost);
               frameIncomingBestEdge.put(targetPoint, connectionData);

               if(targetPoint.equals(goalPoint))
               {
                  break stackLoop;
               }
               else
               {
                  frameStack.add(targetPoint);
               }
            }
         }
      }

      if(frameNodeCosts.containsKey(goalPoint))
      {
         return getPathToPoint(goalPoint);
      }
      else
      {
         return getPathToPoint(closestPointToGoal);
      }
   }

   private List<FramePoint3DReadOnly> getPathToPoint(FrameConnectionPoint3DReadOnly point)
   {
      List<FramePoint3DReadOnly> path = new ArrayList<FramePoint3DReadOnly>(){{add(point);}};

      FrameConnectionData incomingEdge = frameIncomingBestEdge.get(point);
      FrameConnectionPoint3DReadOnly previousTargetPoint = new FrameConnectionPoint3D(point);

      while(incomingEdge != null)
      {
         FrameConnection connection = incomingEdge.connection;
         FrameConnectionPoint3DReadOnly targetPoint = connection.getOppositePoint(previousTargetPoint);
         path.add(targetPoint);
         incomingEdge = frameIncomingBestEdge.get(targetPoint);

         previousTargetPoint = targetPoint;
      }

      Collections.reverse(path);
      return path;
   }
}

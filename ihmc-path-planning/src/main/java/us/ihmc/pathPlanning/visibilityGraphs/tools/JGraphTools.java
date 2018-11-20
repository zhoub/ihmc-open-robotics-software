package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.FrameVisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

public class JGraphTools
{
   public static VisibilityGraphPathPlanner getJGraphPlanner()
   {
      return new VisibilityGraphPathPlanner()
      {
         @Override
         public List<Point3DReadOnly> calculatePath(ConnectionPoint3D start, ConnectionPoint3D goal, Collection<VisibilityMapHolder> visibilityMapHolders)
         {
            return calculatePathOnVisibilityGraph(start, goal, visibilityMapHolders);
         }

         @Override
         public List<FramePoint3DReadOnly> calculatePath(FrameConnectionPoint3DReadOnly start, FrameConnectionPoint3DReadOnly goal,
                                                         Collection<FrameVisibilityMapHolder> visibilityMapHolders)
         {
            return calculatePathOnVisibilityGraph(start, goal, visibilityMapHolders);
         }
      };
   }

   public static List<Point3DReadOnly> calculatePathOnVisibilityGraph(ConnectionPoint3D start, ConnectionPoint3D goal,
                                                                      Collection<VisibilityMapHolder> allVisibilityMapHolders)
   {
      SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graph = createGlobalVisibilityGraph(allVisibilityMapHolders);
      try
      {
         List<DefaultWeightedEdge> solution = DijkstraShortestPath.findPathBetween(graph, start, goal);
         return convertVisibilityGraphSolutionToPath(solution, start, graph);
      }
      catch (IllegalArgumentException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   public static List<FramePoint3DReadOnly> calculatePathOnVisibilityGraph(FrameConnectionPoint3DReadOnly start, FrameConnectionPoint3DReadOnly goal,
                                                                           Collection<FrameVisibilityMapHolder> allVisibilityMapHolders)
   {
      SimpleWeightedGraph<FrameConnectionPoint3DReadOnly, DefaultWeightedEdge> graph = createGlobalFrameVisibilityGraph(allVisibilityMapHolders);
      try
      {
         List<DefaultWeightedEdge> solution = DijkstraShortestPath.findPathBetween(graph, start, goal);
         return convertFrameVisibilityGraphSolutionToPath(solution, start, graph);
      }
      catch (IllegalArgumentException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   public static List<FramePoint3DReadOnly> convertFrameVisibilityGraphSolutionToPath(List<DefaultWeightedEdge> solution, FrameConnectionPoint3DReadOnly start,
                                                                                      Graph<FrameConnectionPoint3DReadOnly, DefaultWeightedEdge> graph)
   {
      if (solution == null)
         return null;

      List<FrameConnection> connections = new ArrayList<>();

      for (DefaultWeightedEdge edge : solution)
      {
         FrameConnectionPoint3DReadOnly source = graph.getEdgeSource(edge);
         FrameConnectionPoint3DReadOnly target = graph.getEdgeTarget(edge);
         connections.add(new FrameConnection(source, target));
      }


      FrameConnectionPoint3DReadOnly previousTarget = start;

      for (FrameConnection connection : connections)
      {
         if (!connection.getSourcePoint().equals(previousTarget))
            connection.flip();
         previousTarget = connection.getTargetPoint();
      }

      // Filter out zigzags
      for (int i = 0; i < connections.size() - 1; i++)
      {
         FrameConnection curr = connections.get(i);
         FrameConnection next = connections.get(i + 1);

         double distanceToNextSquared = next.distanceSquared(curr.getSourcePoint());

         if (distanceToNextSquared < 1.0e-5)
         {
            FrameConnectionPoint3DReadOnly newPoint = next.orthogonalProjection(curr.getSourcePoint(), curr.getSourcePoint().getRegionId());
            connections.set(i, new FrameConnection(curr.getSourcePoint(), newPoint));
            connections.set(i + 1, new FrameConnection(newPoint, next.getTargetPoint()));
         }
      }

      List<FramePoint3DReadOnly> path = new ArrayList<>();
      path.add(start);
      connections.forEach(connection -> path.add(connection.getTargetPoint()));

      return path;
   }

   public static SimpleWeightedGraph<FrameConnectionPoint3DReadOnly, DefaultWeightedEdge> createGlobalFrameVisibilityGraph(Collection<FrameVisibilityMapHolder> allVisibilityMapHolders)
   {
      SimpleWeightedGraph<FrameConnectionPoint3DReadOnly, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

      for (FrameVisibilityMapHolder visibilityMapHolder : allVisibilityMapHolders)
      {
         FrameVisibilityMap visibilityMap = visibilityMapHolder.getVisibilityMap();
         for (FrameConnection connection : visibilityMap)
            addFrameConnectionToGraph(connection, visibilityMapHolder.getConnectionWeight(connection), globalVisMap);
      }

      return globalVisMap;
   }

   public static void addFrameConnectionToGraph(FrameConnection connection, SimpleWeightedGraph<FrameConnectionPoint3DReadOnly, DefaultWeightedEdge> graphToUpdate)
   {
      addFrameConnectionToGraph(connection, connection.length(), graphToUpdate);
   }

   public static void addFrameConnectionToGraph(FrameConnection connection, double connectionWeight,
                                           SimpleWeightedGraph<FrameConnectionPoint3DReadOnly, DefaultWeightedEdge> graphToUpdate)
   {
      FrameConnectionPoint3DReadOnly source = connection.getSourcePoint();
      FrameConnectionPoint3DReadOnly target = connection.getTargetPoint();

      if (!source.epsilonEquals(target, 1.0e-3))
      {
         graphToUpdate.addVertex(source);
         graphToUpdate.addVertex(target);
         DefaultWeightedEdge edge = new DefaultWeightedEdge();
         graphToUpdate.addEdge(source, target, edge);
         graphToUpdate.setEdgeWeight(edge, connectionWeight);
      }
   }

   public static void addFrameConnectionsToGraph(Iterable<FrameConnection> connections, SimpleWeightedGraph<FrameConnectionPoint3DReadOnly, DefaultWeightedEdge> graphToUpdate)
   {
      connections.forEach(connection -> addFrameConnectionToGraph(connection, graphToUpdate));
   }

   public static List<Point3DReadOnly> convertVisibilityGraphSolutionToPath(List<DefaultWeightedEdge> solution, ConnectionPoint3D start,
                                                                            Graph<ConnectionPoint3D, DefaultWeightedEdge> graph)
   {
      if (solution == null)
         return null;

      List<Connection> connections = new ArrayList<>();

      for (DefaultWeightedEdge edge : solution)
      {
         ConnectionPoint3D source = graph.getEdgeSource(edge);
         ConnectionPoint3D target = graph.getEdgeTarget(edge);
         connections.add(new Connection(source, target));
      }


      ConnectionPoint3D previousTarget = start;

      for (Connection connection : connections)
      {
         if (!connection.getSourcePoint().equals(previousTarget))
            connection.flip();
         previousTarget = connection.getTargetPoint();
      }

      // Filter out zigzags
      for (int i = 0; i < connections.size() - 1; i++)
      {
         Connection curr = connections.get(i);
         Connection next = connections.get(i + 1);

         double distanceToNextSquared = next.distanceSquared(curr.getSourcePoint());

         if (distanceToNextSquared < 1.0e-5)
         {
            ConnectionPoint3D newPoint = next.orthogonalProjection(curr.getSourcePoint(), curr.getSourcePoint().getRegionId());
            connections.set(i, new Connection(curr.getSourcePoint(), newPoint));
            connections.set(i + 1, new Connection(newPoint, next.getTargetPoint()));
         }
      }

      List<Point3DReadOnly> path = new ArrayList<>();
      path.add(start);
      connections.forEach(connection -> path.add(connection.getTargetPoint()));

      return path;
   }

   public static void addConnectionToGraph(Connection connection, SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graphToUpdate)
   {
      addConnectionToGraph(connection, connection.length(), graphToUpdate);
   }

   public static void addConnectionToGraph(Connection connection, double connectionWeight, SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graphToUpdate)
   {
      ConnectionPoint3D source = connection.getSourcePoint();
      ConnectionPoint3D target = connection.getTargetPoint();

      if (!source.epsilonEquals(target, 1.0e-3))
      {
         graphToUpdate.addVertex(source);
         graphToUpdate.addVertex(target);
         DefaultWeightedEdge edge = new DefaultWeightedEdge();
         graphToUpdate.addEdge(source, target, edge);
         graphToUpdate.setEdgeWeight(edge, connectionWeight);
      }
   }

   public static void addConnectionsToGraph(Iterable<Connection> connections, SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graphToUpdate)
   {
      connections.forEach(connection -> addConnectionToGraph(connection, graphToUpdate));
   }

   public static SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> createGlobalVisibilityGraph(Collection<VisibilityMapHolder> allVisibilityMapHolders)
   {
      SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

      for (VisibilityMapHolder visibilityMapHolder : allVisibilityMapHolders)
      {
         VisibilityMap visibilityMap = visibilityMapHolder.getVisibilityMapInWorld();
         for (Connection connection : visibilityMap)
            addConnectionToGraph(connection, visibilityMapHolder.getConnectionWeight(connection), globalVisMap);
      }

      return globalVisMap;
   }

   public static SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> createGraphFromConnections(Iterable<Connection> connections)
   {
      SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graph = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);
      addConnectionsToGraph(connections, graph);
      return graph;
   }
}

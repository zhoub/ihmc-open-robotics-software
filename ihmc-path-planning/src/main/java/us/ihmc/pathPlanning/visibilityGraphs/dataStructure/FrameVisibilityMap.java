package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.transform.interfaces.Transform;

import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

public class FrameVisibilityMap implements Iterable<FrameConnection>, FrameChangeable
{

   private Set<FrameConnection> connections;
   private final HashSet<FrameConnectionPoint3D> vertices;

   public FrameVisibilityMap()
   {
      connections = new HashSet<>();
      vertices = new HashSet<>();
   }

   public FrameVisibilityMap(Collection<FrameConnection> connections)
   {
      this();
      setConnections(connections);
      computeVertices();
   }

   public FrameVisibilityMap(FrameVisibilityMap other)
   {
      this();
      set(other);
   }

   public void set(FrameVisibilityMap other)
   {
      setConnections(other.connections);
      computeVertices();
   }

   public void copy(FrameVisibilityMap other)
   {
      copyConnections(other.connections);
      computeVertices();
   }

   public void copyConnections(Collection<FrameConnection> connections)
   {
      this.connections.clear();
      for (FrameConnection connection : connections)
         this.connections.add(connection.getCopy());
   }

   public void setConnections(Collection<FrameConnection> connections)
   {
      this.connections = new HashSet<>(connections);
   }

   public void setConnections(Set<FrameConnection> connections)
   {
      this.connections = connections;
   }

   public void addConnection(FrameConnection connection)
   {
      connections.add(connection);
   }

   public void addConnections(Set<FrameConnection> connections)
   {
      this.connections.addAll(connections);
   }

   public void computeVertices()
   {
      vertices.clear();
      for (FrameConnection connection : connections)
      {
         vertices.add(connection.getSourcePoint());
         vertices.add(connection.getTargetPoint());
      }
   }

   public Set<FrameConnectionPoint3D> getVertices()
   {
      return vertices;
   }

   public Set<FrameConnection> getConnections()
   {
      return connections;
   }



   public boolean isEmpty()
   {
      return connections.isEmpty();
   }

   @Override
   public Iterator<FrameConnection> iterator()
   {
      return connections.iterator();
   }

}

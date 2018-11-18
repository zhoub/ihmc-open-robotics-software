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
   private final HashSet<FrameConnectionPoint3DReadOnly> vertices;

   private ReferenceFrame referenceFrame;

   public FrameVisibilityMap(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      connections = new HashSet<>();
      vertices = new HashSet<>();
   }

   public FrameVisibilityMap(ReferenceFrame referenceFrame, Collection<FrameConnection> connections)
   {
      this(referenceFrame);
      setConnections(connections);
      computeVertices();
   }

   public FrameVisibilityMap(FrameVisibilityMap other)
   {
      this(other.getReferenceFrame());
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
      changeFrameInternal();
   }

   public void setConnections(Set<FrameConnection> connections)
   {
      this.connections = connections;
      changeFrameInternal();
   }

   public void addConnection(FrameConnection connection)
   {
      connection.checkReferenceFrameMatch(referenceFrame);
      connections.add(connection);
   }

   public void addConnections(Set<FrameConnection> connections)
   {
      for (FrameConnection connection : connections)
         connection.checkReferenceFrameMatch(referenceFrame);
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

   public Set<FrameConnectionPoint3DReadOnly> getVertices()
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

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      for (FrameConnection connection : connections)
         connection.applyTransform(transform);
      computeVertices();
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (FrameConnection connection : connections)
         connection.applyInverseTransform(transform);
      computeVertices();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return null;
   }

   private void changeFrameInternal()
   {
      for (FrameConnection connection : connections)
         connection.changeFrame(referenceFrame);
   }
}

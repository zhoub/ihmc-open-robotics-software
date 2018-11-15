package us.ihmc.pathPlanning.visibilityGraphs.ui.viewers;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import javafx.animation.AnimationTimer;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisualizationParameters;

public class ClusterMeshViewer extends AnimationTimer
{
   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private final Group root = new Group();
   private final Group rawPointsGroup = new Group();
   private final Group navigableExtrusionsGroup = new Group();
   private final Group nonNavigableExtrusionsGroup = new Group();
   private final Group rotationExtrusionsGroup = new Group();
   private final AtomicReference<Map<Integer, MeshView>> rawPointsToRenderReference = new AtomicReference<>(null);
   private final AtomicReference<Map<Integer, MeshView>> navigableExtrusionsToRenderReference = new AtomicReference<>(null);
   private final AtomicReference<Map<Integer, MeshView>> nonNavigableExtrusionsToRenderReference = new AtomicReference<>(null);
   private final AtomicReference<Map<Integer, MeshView>> rotationExtrusionsToRenderReference = new AtomicReference<>(null);

   private AtomicReference<Boolean> resetRequested;
   private AtomicReference<Boolean> showRawPoints;
   private AtomicReference<Boolean> showNavigableExtrusions;
   private AtomicReference<Boolean> showNonNavigableExtrusions;
   private AtomicReference<Boolean> showRotationExtrusions;

   private AtomicReference<List<NavigableRegion>> newRequestReference;

   private final Messager messager;

   public ClusterMeshViewer(Messager messager)
   {
      this(messager, null);
   }

   public ClusterMeshViewer(Messager messager, ExecutorService executorService)
   {
      this.messager = messager;

      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;

      root.setMouseTransparent(true);
      root.getChildren().addAll(rawPointsGroup, navigableExtrusionsGroup, nonNavigableExtrusionsGroup, rotationExtrusionsGroup);
   }

   public void setTopics(Topic<Boolean> resetRequestedTopic, Topic<Boolean> showClusterRawPointsTopic, Topic<Boolean> showClusterNavigableExtrusionsTopic,
                         Topic<Boolean> showClusterNonNavigableExtrusionsTopic, Topic<List<NavigableRegion>> navigableRegionDataTopic,
                         Topic<Boolean> showClusterRotationExtrusionsTopic)
   {
      resetRequested = messager.createInput(resetRequestedTopic, false);
      showRawPoints = messager.createInput(showClusterRawPointsTopic, false);
      showNavigableExtrusions = messager.createInput(showClusterNavigableExtrusionsTopic, false);
      showNonNavigableExtrusions = messager.createInput(showClusterNonNavigableExtrusionsTopic, false);
      showRotationExtrusions = messager.createInput(showClusterRotationExtrusionsTopic, false);
      newRequestReference = messager.createInput(navigableRegionDataTopic, null);
   }

   @Override
   public void handle(long now)
   {
      if (resetRequested.getAndSet(false))
      {
         rawPointsGroup.getChildren().clear();
         rawPointsToRenderReference.getAndSet(null);
         navigableExtrusionsGroup.getChildren().clear();
         navigableExtrusionsToRenderReference.getAndSet(null);
         nonNavigableExtrusionsGroup.getChildren().clear();
         nonNavigableExtrusionsToRenderReference.getAndSet(null);
         return;
      }

      Map<Integer, MeshView> rawPointsToRender = rawPointsToRenderReference.get();

      if (rawPointsToRender != null)
      {
         rawPointsGroup.getChildren().clear();

         if (showRawPoints.get())
            rawPointsGroup.getChildren().addAll(rawPointsToRender.values());
      }

      Map<Integer, MeshView> navigableExtrusionsRender = navigableExtrusionsToRenderReference.get();

      if (navigableExtrusionsRender != null)
      {
         navigableExtrusionsGroup.getChildren().clear();

         if (showNavigableExtrusions.get())
            navigableExtrusionsGroup.getChildren().addAll(navigableExtrusionsRender.values());
      }

      Map<Integer, MeshView> nonNavigableExtrusionsRender = nonNavigableExtrusionsToRenderReference.get();

      if (nonNavigableExtrusionsRender != null)
      {
         nonNavigableExtrusionsGroup.getChildren().clear();

         if (showNonNavigableExtrusions.get())
            nonNavigableExtrusionsGroup.getChildren().addAll(nonNavigableExtrusionsRender.values());
      }

      Map<Integer, MeshView> rotationExtrusionsRender = rotationExtrusionsToRenderReference.get();

      if (rotationExtrusionsRender != null)
      {
         rotationExtrusionsGroup.getChildren().clear();

         if (showRotationExtrusions.get())
            rotationExtrusionsGroup.getChildren().addAll(rotationExtrusionsRender.values());
      }

      if (showRawPoints.get() || showNavigableExtrusions.get() || showNonNavigableExtrusions.get() || showRotationExtrusions.get())
      {
         List<NavigableRegion> newRequest = newRequestReference.getAndSet(null);

         if (newRequest != null)
            processNavigableRegionsOnThread(newRequest);
      }
   }

   private void processNavigableRegionsOnThread(List<NavigableRegion> navigableRegionLocalPlanners)
   {
      executorService.execute(() -> processNavigableRegions(navigableRegionLocalPlanners));
   }

   private void processNavigableRegions(List<NavigableRegion> navigableRegionLocalPlanners)
   {
      Map<Integer, JavaFXMeshBuilder> rawPointsMeshBuilders = new HashMap<>();
      Map<Integer, JavaFXMeshBuilder> navigableExtrusionsMeshBuilders = new HashMap<>();
      Map<Integer, JavaFXMeshBuilder> nonNavigableExtrusionsMeshBuilders = new HashMap<>();
      Map<Integer, JavaFXMeshBuilder> rotationExtrusionsMeshBuilders = new HashMap<>();
      Map<Integer, Material> navigableMaterials = new HashMap<>();
      Map<Integer, Material> nonNavigableMaterials = new HashMap<>();
      Map<Integer, Material> rotationMaterials = new HashMap<>();

      for (NavigableRegion navigableRegionLocalPlanner : navigableRegionLocalPlanners)
      {
         int regionId = navigableRegionLocalPlanner.getMapId();
         JavaFXMeshBuilder rawPointsMeshBuilder = getOrCreate(rawPointsMeshBuilders, regionId);
         JavaFXMeshBuilder navigableExtrusionsMeshBuilder = getOrCreate(navigableExtrusionsMeshBuilders, regionId);
         JavaFXMeshBuilder nonNavigableExtrusionsMeshBuilder = getOrCreate(nonNavigableExtrusionsMeshBuilders, regionId);
         JavaFXMeshBuilder rotationExtrusionsMeshBuilder = getOrCreate(rotationExtrusionsMeshBuilders, regionId);

         List<Cluster> clusters = navigableRegionLocalPlanner.getAllClusters();
         for (Cluster cluster : clusters)
         {
            for (Point3DReadOnly rawPoint : cluster.getRawPointsInWorld())
               rawPointsMeshBuilder.addTetrahedron(VisualizationParameters.CLUSTER_RAWPOINT_SIZE, rawPoint);
            navigableExtrusionsMeshBuilder
                  .addMultiLine(cluster.getNavigableExtrusionsInWorld(), VisualizationParameters.NAVIGABLECLUSTER_LINE_THICKNESS, false);
            nonNavigableExtrusionsMeshBuilder
                  .addMultiLine(cluster.getNonNavigableExtrusionsInWorld(), VisualizationParameters.NON_NAVIGABLECLUSTER_LINE_THICKNESS, false);
            rotationExtrusionsMeshBuilder
                  .addMultiLine(cluster.getRotationExtrusionsInWorld(), VisualizationParameters.ROTATIONCLUSTER_LINE_THICKNESS, false);
         }

         navigableMaterials.put(regionId, new PhongMaterial(getNavigableLineColor(regionId)));
         nonNavigableMaterials.put(regionId, new PhongMaterial(getNonNavigableLineColor(regionId)));
      }

      HashMap<Integer, MeshView> rawPointsMapToRender = new HashMap<>();
      HashMap<Integer, MeshView> navigableExtrusionsMapToRender = new HashMap<>();
      HashMap<Integer, MeshView> nonNavigableExtrusionsMapToRender = new HashMap<>();
      HashMap<Integer, MeshView> rotationExtrusionsMapToRender = new HashMap<>();

      for (Integer id : rawPointsMeshBuilders.keySet())
      {
         MeshView rawPointsMeshView = new MeshView(rawPointsMeshBuilders.get(id).generateMesh());
         rawPointsMeshView.setMaterial(nonNavigableMaterials.get(id));
         rawPointsMapToRender.put(id, rawPointsMeshView);

         MeshView navigableExtrusionsMeshView = new MeshView(navigableExtrusionsMeshBuilders.get(id).generateMesh());
         navigableExtrusionsMeshView.setMaterial(navigableMaterials.get(id));
         navigableExtrusionsMapToRender.put(id, navigableExtrusionsMeshView);

         MeshView nonNavigableExtrusionsMeshView = new MeshView(nonNavigableExtrusionsMeshBuilders.get(id).generateMesh());
         nonNavigableExtrusionsMeshView.setMaterial(nonNavigableMaterials.get(id));
         nonNavigableExtrusionsMapToRender.put(id, nonNavigableExtrusionsMeshView);

         MeshView rotationExtrusionsMeshView = new MeshView(rotationExtrusionsMeshBuilders.get(id).generateMesh());
         rotationExtrusionsMeshView.setMaterial(rotationMaterials.get(id));
         rotationExtrusionsMapToRender.put(id, rotationExtrusionsMeshView);
      }

      rawPointsToRenderReference.set(rawPointsMapToRender);
      navigableExtrusionsToRenderReference.set(navigableExtrusionsMapToRender);
      nonNavigableExtrusionsToRenderReference.set(nonNavigableExtrusionsMapToRender);
      rotationExtrusionsToRenderReference.set(rotationExtrusionsMapToRender);
   }

   private JavaFXMeshBuilder getOrCreate(Map<Integer, JavaFXMeshBuilder> meshBuilders, int regionId)
   {
      JavaFXMeshBuilder meshBuilder = meshBuilders.get(regionId);
      if (meshBuilder == null)
      {
         meshBuilder = new JavaFXMeshBuilder();
         meshBuilders.put(regionId, meshBuilder);
      }
      return meshBuilder;
   }

   private Color getNonNavigableLineColor(int regionId)
   {
      return PlanarRegionViewer.getRegionColor(regionId).darker();
   }

   private Color getNavigableLineColor(int regionId)
   {
      return PlanarRegionViewer.getRegionColor(regionId).brighter();
   }

   @Override
   public void stop()
   {
      super.stop();
      if (!isExecutorServiceProvided)
         executorService.shutdownNow();
   }

   public Node getRoot()
   {
      return root;
   }
}

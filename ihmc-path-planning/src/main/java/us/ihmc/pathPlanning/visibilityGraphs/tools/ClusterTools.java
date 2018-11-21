package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.FrameCluster;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.commons.lists.ListWrappingIndexTools;

public class ClusterTools
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double HALF_PI = 0.5 * Math.PI;
   private static final double POPPING_POLYGON_POINTS_THRESHOLD = 0.0; //MathTools.square(0.025);
   private static final double POPPING_MULTILINE_POINTS_THRESHOLD = MathTools.square(0.20);
   private static final double NAV_TO_NON_NAV_DISTANCE = 0.001;

   public static List<FramePoint2DBasics> extrudePolygonWithFrames(boolean extrudeToTheLeft, FrameCluster cluster,
                                                                   ObstacleExtrusionDistanceCalculator calculator)
   {
      List<? extends FramePoint3DReadOnly> rawPoints = cluster.getRawPoints();
      List<? extends FramePoint2DReadOnly> rawPoints2D = rawPoints.stream().map(point -> new FramePoint2D(point)).collect(Collectors.toList());

      double[] extrusionDistances = rawPoints.stream().mapToDouble(rawPoint -> calculator.computeExtrusionDistance(new Point2D(rawPoint), rawPoint.getZ()))
                                             .toArray();

      return extrudePolygonWithFrames(extrudeToTheLeft, rawPoints2D, extrusionDistances);
   }

   static List<FramePoint2DBasics> extrudePolygonWithFrames(boolean extrudeToTheLeft, List<? extends FramePoint2DReadOnly> pointsToExtrude,
                                                            double[] extrusionDistances)
   {
      if (pointsToExtrude.size() == 2)
      {
         return extrudeMultiLineWithFrames(pointsToExtrude, extrusionDistances, 5);
      }

      // check frames
      ReferenceFrame referenceFrame = pointsToExtrude.get(0).getReferenceFrame();
      for (int i = 1; i < pointsToExtrude.size(); i++)
      {
         pointsToExtrude.get(i).checkReferenceFrameMatch(referenceFrame);
      }

      List<FramePoint2DBasics> extrusions = new ArrayList<>();

      for (int i = 0; i < pointsToExtrude.size(); i++)
      {
         FramePoint2DReadOnly previousPoint = ListWrappingIndexTools.getPrevious(i, pointsToExtrude);
         FramePoint2DReadOnly pointToExtrude = pointsToExtrude.get(i);

         if (pointToExtrude.distanceSquared(previousPoint) < POPPING_POLYGON_POINTS_THRESHOLD)
            continue;

         FramePoint2DReadOnly nextPoint = ListWrappingIndexTools.getNext(i, pointsToExtrude);

         double extrusionDistance = extrusionDistances[i];

         FrameLine2D edgePrev = new FrameLine2D(referenceFrame, previousPoint, pointToExtrude);
         FrameLine2D edgeNext = new FrameLine2D(referenceFrame, pointToExtrude, nextPoint);

         boolean shouldExtrudeCorner;

         if (extrudeToTheLeft)
            shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -HALF_PI;
         else
            shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) >= HALF_PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeCornerWithFrames(pointToExtrude, edgePrev, edgeNext, extrudeToTheLeft, 3, extrusionDistance));
         }
         else
         {
            FrameVector2D extrusionDirection = new FrameVector2D(referenceFrame);
            extrusionDirection.interpolate(edgePrev.getDirection(), edgeNext.getDirection(), 0.5);
            extrusionDirection = EuclidFrameTools.perpendicularVector2D(extrusionDirection);
            if (!extrudeToTheLeft)
               extrusionDirection.negate();

            FramePoint2D extrusion = new FramePoint2D(referenceFrame);
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, pointToExtrude);
            extrusions.add(extrusion);
         }
      }

      if (!extrusions.isEmpty())
         extrusions.add(extrusions.get(0));

      return extrusions;
   }

   public static List<FramePoint2DBasics> extrudeMultiLineWithFrames(FrameCluster cluster, ObstacleExtrusionDistanceCalculator calculator,
                                                                     int numberOfExtrusionsAtEndpoints)
   {
      List<? extends FramePoint3DReadOnly> rawPoints = cluster.getRawPoints();
      List<? extends FramePoint2DReadOnly> rawPoints2D = rawPoints.stream().map(point -> new FramePoint2D(point)).collect(Collectors.toList());
      double[] extrusionDistances = rawPoints.stream().mapToDouble(rawPoint -> calculator.computeExtrusionDistance(new Point2D(rawPoint), rawPoint.getZ()))
                                             .toArray();

      return extrudeMultiLineWithFrames(rawPoints2D, extrusionDistances, numberOfExtrusionsAtEndpoints);
   }

   private static List<FramePoint2DBasics> extrudeMultiLineWithFrames(List<? extends FramePoint2DReadOnly> pointsToExtrude, double[] extrusionDistances,
                                                                      int numberOfExtrusionsAtEndpoints)
   {
      List<FramePoint2DBasics> extrusions = new ArrayList<>();

      // check frames
      ReferenceFrame referenceFrame = pointsToExtrude.get(0).getReferenceFrame();
      for (int i = 1; i < pointsToExtrude.size(); i++)
      {
         pointsToExtrude.get(i).checkReferenceFrameMatch(referenceFrame);
      }

      if (pointsToExtrude.size() >= 2)
      { // Start
         FramePoint2DReadOnly pointToExtrude = pointsToExtrude.get(0);
         FramePoint2DReadOnly nextPointToExtrude = pointsToExtrude.get(1);

         double extrusionDistance = extrusionDistances[0];

         FrameLine2D edgePrev = new FrameLine2D(referenceFrame, nextPointToExtrude, pointToExtrude);
         FrameLine2D edgeNext = new FrameLine2D(referenceFrame, pointToExtrude, nextPointToExtrude);
         extrusions.addAll(extrudeCornerWithFrames(pointToExtrude, edgePrev, edgeNext, true, numberOfExtrusionsAtEndpoints, extrusionDistance));
      }

      for (int i = 1; i < pointsToExtrude.size() - 1; i++)
      { // Go from start to end
         FramePoint2DReadOnly pointToExtrude = pointsToExtrude.get(i);

         double extrusionDistance = extrusionDistances[i];

         FrameLine2D edgePrev = new FrameLine2D(referenceFrame, pointsToExtrude.get(i - 1), pointToExtrude);
         FrameLine2D edgeNext = new FrameLine2D(referenceFrame, pointToExtrude, pointsToExtrude.get(i + 1));

         boolean shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -HALF_PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeCornerWithFrames(pointToExtrude, edgePrev, edgeNext, true, 3, extrusionDistance));
         }
         else
         {
            FrameVector2D extrusionDirection = new FrameVector2D(referenceFrame);
            extrusionDirection.interpolate(edgePrev.getDirection(), edgeNext.getDirection(), 0.5);
            extrusionDirection.normalize();
            extrusionDirection = EuclidFrameTools.perpendicularVector2D(extrusionDirection);

            FramePoint2D extrusion = new FramePoint2D(referenceFrame);
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, pointToExtrude);
            extrusions.add(extrusion);
         }
      }

      if (pointsToExtrude.size() >= 2)
      { // End
         int lastIndex = pointsToExtrude.size() - 1;
         FramePoint2DReadOnly pointToExtrude = pointsToExtrude.get(lastIndex);
         double extrusionDistance = extrusionDistances[lastIndex];

         FrameLine2D edgePrev = new FrameLine2D(referenceFrame, pointsToExtrude.get(lastIndex - 1), pointToExtrude);
         FrameLine2D edgeNext = new FrameLine2D(referenceFrame, pointToExtrude, pointsToExtrude.get(lastIndex - 1));
         extrusions.addAll(extrudeCornerWithFrames(pointToExtrude, edgePrev, edgeNext, true, numberOfExtrusionsAtEndpoints, extrusionDistance));
      }

      for (int i = pointsToExtrude.size() - 2; i >= 1; i--)
      { // Go from end back to start
         FramePoint2DReadOnly pointToExtrude = pointsToExtrude.get(i);
         double extrusionDistance = extrusionDistances[i];

         FrameLine2D edgePrev = new FrameLine2D(referenceFrame, pointsToExtrude.get(i + 1), pointToExtrude);
         FrameLine2D edgeNext = new FrameLine2D(referenceFrame, pointToExtrude, pointsToExtrude.get(i - 1));

         boolean shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -HALF_PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeCornerWithFrames(pointToExtrude, edgePrev, edgeNext, true, 3, extrusionDistance));
         }
         else
         {
            FrameVector2D extrusionDirection = new FrameVector2D(referenceFrame);
            extrusionDirection.interpolate(edgePrev.getDirection(), edgeNext.getDirection(), 0.5);
            extrusionDirection.normalize();
            extrusionDirection = EuclidFrameTools.perpendicularVector2D(extrusionDirection);

            FramePoint2D extrusion = new FramePoint2D(referenceFrame);
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, pointToExtrude);
            extrusions.add(extrusion);
         }
      }

      if (!extrusions.isEmpty())
         extrusions.add(extrusions.get(0));

      return extrusions;
   }

   public static List<FramePoint2DBasics> extrudeCornerWithFrames(FramePoint2DReadOnly cornerPointToExtrude, FrameLine2D previousEdge, FrameLine2D nextEdge,
                                                                  boolean extrudeToTheLeft, int numberOfExtrusions, double extrusionDistance)
   {
      ReferenceFrame referenceFrame = cornerPointToExtrude.getReferenceFrame();
      previousEdge.checkReferenceFrameMatch(referenceFrame);
      nextEdge.checkReferenceFrameMatch(referenceFrame);

      List<FramePoint2DBasics> extrusions = new ArrayList<>();

      FrameVector2D firstExtrusionDirection = EuclidFrameTools.perpendicularVector2D(previousEdge.getDirection());
      if (!extrudeToTheLeft)
         firstExtrusionDirection.negate();
      FramePoint2D firstExtrusion = new FramePoint2D(referenceFrame);
      firstExtrusion.scaleAdd(extrusionDistance, firstExtrusionDirection, cornerPointToExtrude);
      extrusions.add(firstExtrusion);

      FrameVector2D lastExtrusionDirection = EuclidFrameTools.perpendicularVector2D(nextEdge.getDirection());
      if (!extrudeToTheLeft)
         lastExtrusionDirection.negate();
      FramePoint2D lastExtrusion = new FramePoint2D(referenceFrame);
      lastExtrusion.scaleAdd(extrusionDistance, lastExtrusionDirection, cornerPointToExtrude);

      if (numberOfExtrusions > 2)
      {
         double openingAngle = firstExtrusionDirection.angle(lastExtrusionDirection);
         if (MathTools.epsilonEquals(Math.PI, Math.abs(openingAngle), 1.0e-7))
            openingAngle = extrudeToTheLeft ? -Math.PI : Math.PI;

         FrameVector2D extrusionDirection = new FrameVector2D(referenceFrame);

         for (int i = 1; i < numberOfExtrusions - 1; i++)
         {
            double alpha = i / (numberOfExtrusions - 1.0);
            RotationMatrixTools.applyYawRotation(alpha * openingAngle, firstExtrusionDirection, extrusionDirection);
            FramePoint2D extrusion = new FramePoint2D(referenceFrame);
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, cornerPointToExtrude);
            extrusions.add(extrusion);
         }
      }

      extrusions.add(lastExtrusion);

      return extrusions;
   }

   public static FrameCluster createHomeRegionFrameCluster(PlanarRegion homeRegion, NavigableExtrusionDistanceCalculator calculator)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      homeRegion.getTransformToWorld(transformToWorld);

      FrameCluster cluster = new FrameCluster(transformToWorld);
      cluster.setType(Type.POLYGON);
      cluster.addRawPoints(cluster.getReferenceFrame(), homeRegion.getConcaveHull());
      cluster.setExtrusionSide(ExtrusionSide.INSIDE);

      double extrusionDistance = calculator.computeExtrusionDistance(homeRegion);

      ObstacleExtrusionDistanceCalculator nonNavigableCalculator = (p, h) -> extrusionDistance - NAV_TO_NON_NAV_DISTANCE;
      ObstacleExtrusionDistanceCalculator navigableCalculator = (p, h) -> extrusionDistance;

      boolean extrudeToTheLeft = cluster.getExtrusionSide() != ExtrusionSide.INSIDE;
      cluster.addNonNavigableExtrusions(extrudePolygonWithFrames(extrudeToTheLeft, cluster, nonNavigableCalculator));
      cluster.addNavigableExtrusions(extrudePolygonWithFrames(extrudeToTheLeft, cluster, navigableCalculator));
      cluster.updateBoundingBox();
      return cluster;
   }

   public static List<FrameCluster> createObstacleFrameClusters(PlanarRegion homeRegion, List<PlanarRegion> obstacleRegions, double orthogonalAngle,
                                                                ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator)
   {
      List<FrameCluster> obstacleClusters = new ArrayList<>();

      RigidBodyTransform transformFromHomeToWorld = new RigidBodyTransform();
      homeRegion.getTransformToWorld(transformFromHomeToWorld);
      Vector3D referenceNormal = homeRegion.getNormal();
      double zThresholdBeforeOrthogonal = Math.cos(orthogonalAngle);

      for (PlanarRegion obstacleRegion : obstacleRegions)
      {
         Vector3D otherNormal = obstacleRegion.getNormal();

         FrameCluster cluster = new FrameCluster(transformFromHomeToWorld);
         cluster.setExtrusionSide(ExtrusionSide.OUTSIDE);

         List<FramePoint3DBasics> rawPoints = new ArrayList<>();
         RigidBodyTransform transformFromOtherToWorld = new RigidBodyTransform();
         obstacleRegion.getTransformToWorld(transformFromOtherToWorld);
         ReferenceFrame otherFrame = new ReferenceFrame("otherFrame", worldFrame)
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.set(transformFromOtherToWorld);
            }
         };
         otherFrame.update();

         for (int i = 0; i < obstacleRegion.getConvexHull().getNumberOfVertices(); i++)
         {
            FramePoint3D concaveHullVertexHome = new FramePoint3D(otherFrame, obstacleRegion.getConvexHull().getVertex(i));
            rawPoints.add(concaveHullVertexHome);
         }

         if (Math.abs(otherNormal.dot(referenceNormal)) < zThresholdBeforeOrthogonal)
         { // Project region as a line
            cluster.setType(Type.MULTI_LINE);
            cluster.addRawPoints(filterVerticalPolygonForMultiLineExtrusionWithFrames(rawPoints, POPPING_MULTILINE_POINTS_THRESHOLD));
         }
         else
         { // Project region as a polygon
            cluster.setType(Type.POLYGON);
            cluster.addRawPoints(rawPoints);
         }

         extrudeObstacleFrameCluster(cluster, extrusionDistanceCalculator);
         obstacleClusters.add(cluster);
      }

      return obstacleClusters;
   }

   public static void extrudeObstacleFrameCluster(FrameCluster cluster, ObstacleExtrusionDistanceCalculator calculator)
   {
      ObstacleExtrusionDistanceCalculator nonNavigableCalculator = (p, h) -> calculator.computeExtrusionDistance(p, h) - NAV_TO_NON_NAV_DISTANCE;
      ObstacleExtrusionDistanceCalculator navigableCalculator = calculator;
      int numberOfExtrusionsAtEndpoints = 5;

      switch (cluster.getType())
      {
      case LINE:
      case MULTI_LINE:
         cluster.addNonNavigableExtrusions(extrudeMultiLineWithFrames(cluster, nonNavigableCalculator, numberOfExtrusionsAtEndpoints));
         cluster.addNavigableExtrusions(extrudeMultiLineWithFrames(cluster, navigableCalculator, numberOfExtrusionsAtEndpoints));
         break;
      case POLYGON:
         boolean extrudeToTheLeft = cluster.getExtrusionSide() != ExtrusionSide.INSIDE;

         try
         {
            cluster.addNonNavigableExtrusions(extrudePolygonWithFrames(extrudeToTheLeft, cluster, nonNavigableCalculator));
         }
         catch (Exception e)
         {
            e.printStackTrace();
            return;
         }

         cluster.addNavigableExtrusions(extrudePolygonWithFrames(extrudeToTheLeft, cluster, navigableCalculator));
         break;

      default:
         throw new RuntimeException("Unhandled cluster type: " + cluster.getType());
      }
      cluster.updateBoundingBox();
   }

   static List<FramePoint3DBasics> filterVerticalPolygonForMultiLineExtrusionWithFrames(List<FramePoint3DBasics> verticalPolygonVertices,
                                                                   double poppingPointsDistanceSquaredThreshold)
   {
      // Making a deep copy
      ReferenceFrame referenceFrame = verticalPolygonVertices.get(0).getReferenceFrame();
      List<FramePoint3DBasics> filteredPoints = verticalPolygonVertices.stream().map(point ->
                                                                                     {
                                                                                        point.checkReferenceFrameMatch(referenceFrame);
                                                                                        return new FramePoint3D(point);
                                                                                     }).collect(Collectors.toList());

      if (verticalPolygonVertices.size() <= 2)
         return filteredPoints;

      FramePoint3D mean = new FramePoint3D(referenceFrame);
      FrameVector3D principalVector = new FrameVector3D(referenceFrame);
      PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

      pca.clear();
      filteredPoints.forEach(p -> pca.addPoint(p.getX(), p.getY(), 0.0));
      pca.compute();
      pca.getMean(mean);
      pca.getPrincipalVector(principalVector);
      FrameLine2D line = new FrameLine2D(new FramePoint2D(mean), new FrameVector2D(principalVector));
      // Projecting the points in the 2D plane described by the z-axis and the line direction
      List<FramePoint2DReadOnly> projectedPoints = new ArrayList<>();
      for (FramePoint3DBasics point : filteredPoints)
      {
         double x = line.parameterGivenPointOnLine(new FramePoint2D(point), Double.POSITIVE_INFINITY);
         double z = point.getZ();
         projectedPoints.add(new FramePoint2D(referenceFrame, x, z));
      }

      for (int pointIndex = 0; pointIndex < projectedPoints.size(); pointIndex++)
      {
         FramePoint3DBasics point = filteredPoints.get(pointIndex);
         FramePoint2DReadOnly projectedPoint = projectedPoints.get(pointIndex);

         for (int edgeIndex = 0; edgeIndex < projectedPoints.size(); edgeIndex++)
         {
            FramePoint2DReadOnly edgeStart = projectedPoints.get(edgeIndex);
            FramePoint2DReadOnly edgeEnd = ListWrappingIndexTools.getNext(edgeIndex, projectedPoints);

            // Check if the point is between start and end
            double signedDistanceToStart = edgeStart.getX() - projectedPoint.getX();
            double signedDistanceToEnd = edgeEnd.getX() - projectedPoint.getX();
            if (signedDistanceToStart * signedDistanceToEnd > 0.0)
               continue; // If same sign, the edge is not above/below the point, keep going.
            // The edge is above or below the point, let's compute the edge height at the point x-coordinate
            double alpha = EuclidFrameTools.percentageAlongLineSegment2D(projectedPoint, edgeStart, edgeEnd);
            double height = EuclidCoreTools.interpolate(edgeStart.getY(), edgeEnd.getY(), alpha);

            point.setZ(Math.max(height, point.getZ()));
         }

         // Adjust the XY-coordinates to be on the line
         point.set(line.pointOnLineGivenParameter(projectedPoint.getX()));
      }

      // Sort the points given their position on the line.
      Collections.sort(filteredPoints, (p1, p2) -> {
         double t1 = line.parameterGivenPointOnLine(new FramePoint2D(p1), Double.POSITIVE_INFINITY);
         double t2 = line.parameterGivenPointOnLine(new FramePoint2D(p2), Double.POSITIVE_INFINITY);
         return t1 >= t2 ? 1 : -1;
      });

      // FIXME Problem with the obstacle height.
      if (filteredPoints.get(0).distanceXYSquared(filteredPoints.get(filteredPoints.size() - 1)) <= poppingPointsDistanceSquaredThreshold)
      {
         double maxHeight = filteredPoints.stream().map(FramePoint3DBasics::getZ).max((d1, d2) -> Double.compare(d1, d2)).get();
         List<FramePoint3DBasics> endpoints = new ArrayList<>();
         endpoints.add(filteredPoints.get(0));
         endpoints.add(filteredPoints.get(filteredPoints.size() - 1));
         endpoints.forEach(p -> p.setZ(maxHeight));
         return endpoints;
      }
      else
      {
         int index = 0;
         // Look for points with same XY-coordinates and only keep the highest one.
         while (index < filteredPoints.size() - 1)
         {
            FramePoint3DReadOnly pointCurr = filteredPoints.get(index);
            FramePoint3DReadOnly pointNext = filteredPoints.get(index + 1);

            if (pointCurr.distanceXYSquared(pointNext) < poppingPointsDistanceSquaredThreshold)
               filteredPoints.remove(pointCurr.getZ() <= pointNext.getZ() ? index : index + 1);
            else
               index++;
         }
         return filteredPoints;
      }
   }

   public static List<Point2D> extrudePolygon(boolean extrudeToTheLeft, Cluster cluster, ObstacleExtrusionDistanceCalculator calculator)
   {
      List<Point2DReadOnly> rawPoints = cluster.getRawPointsInLocal2D();
      double[] extrusionDistances = cluster.getRawPointsInLocal3D().stream()
                                           .mapToDouble(rawPoint -> calculator.computeExtrusionDistance(new Point2D(rawPoint), rawPoint.getZ())).toArray();

      return extrudePolygon(extrudeToTheLeft, rawPoints, extrusionDistances);
   }

   static List<Point2D> extrudePolygon(boolean extrudeToTheLeft, List<Point2DReadOnly> pointsToExtrude, double[] extrusionDistances)
   {
      if (pointsToExtrude.size() == 2)
      {
         return extrudeMultiLine(pointsToExtrude, extrusionDistances, 5);
      }

      List<Point2D> extrusions = new ArrayList<>();

      for (int i = 0; i < pointsToExtrude.size(); i++)
      {
         Point2DReadOnly previousPoint = ListWrappingIndexTools.getPrevious(i, pointsToExtrude);
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(i);

         if (pointToExtrude.distanceSquared(previousPoint) < POPPING_POLYGON_POINTS_THRESHOLD)
            continue;

         Point2DReadOnly nextPoint = ListWrappingIndexTools.getNext(i, pointsToExtrude);

         double extrusionDistance = extrusionDistances[i];

         Line2D edgePrev = new Line2D(previousPoint, pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, nextPoint);

         boolean shouldExtrudeCorner;

         if (extrudeToTheLeft)
            shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -HALF_PI;
         else
            shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) >= HALF_PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeCorner(pointToExtrude, edgePrev, edgeNext, extrudeToTheLeft, 3, extrusionDistance));
         }
         else
         {
            Vector2D extrusionDirection = new Vector2D();
            extrusionDirection.interpolate(edgePrev.getDirection(), edgeNext.getDirection(), 0.5);
            extrusionDirection = EuclidGeometryTools.perpendicularVector2D(extrusionDirection);
            if (!extrudeToTheLeft)
               extrusionDirection.negate();

            Point2D extrusion = new Point2D();
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, pointToExtrude);
            extrusions.add(extrusion);
         }
      }

      if (!extrusions.isEmpty())
         extrusions.add(extrusions.get(0));

      return extrusions;
   }

   public static List<Point2D> extrudeMultiLine(Cluster cluster, ObstacleExtrusionDistanceCalculator calculator, int numberOfExtrusionsAtEndpoints)
   {
      List<Point2DReadOnly> rawPoints = cluster.getRawPointsInLocal2D();
      double[] extrusionDistances = cluster.getRawPointsInLocal3D().stream()
                                           .mapToDouble(rawPoint -> calculator.computeExtrusionDistance(new Point2D(rawPoint), rawPoint.getZ())).toArray();

      return extrudeMultiLine(rawPoints, extrusionDistances, numberOfExtrusionsAtEndpoints);
   }

   private static List<Point2D> extrudeMultiLine(List<Point2DReadOnly> pointsToExtrude, double[] extrusionDistances, int numberOfExtrusionsAtEndpoints)
   {
      List<Point2D> extrusions = new ArrayList<>();

      if (pointsToExtrude.size() >= 2)
      { // Start
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(0);
         double extrusionDistance = extrusionDistances[0];

         Line2D edgePrev = new Line2D(pointsToExtrude.get(1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, pointsToExtrude.get(1));
         extrusions.addAll(extrudeCorner(pointToExtrude, edgePrev, edgeNext, true, numberOfExtrusionsAtEndpoints, extrusionDistance));
      }

      for (int i = 1; i < pointsToExtrude.size() - 1; i++)
      { // Go from start to end
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(i);
         double extrusionDistance = extrusionDistances[i];

         Line2D edgePrev = new Line2D(pointsToExtrude.get(i - 1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, pointsToExtrude.get(i + 1));

         boolean shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -HALF_PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeCorner(pointToExtrude, edgePrev, edgeNext, true, 3, extrusionDistance));
         }
         else
         {
            Vector2D extrusionDirection = new Vector2D();
            extrusionDirection.interpolate(edgePrev.getDirection(), edgeNext.getDirection(), 0.5);
            extrusionDirection.normalize();
            extrusionDirection = EuclidGeometryTools.perpendicularVector2D(extrusionDirection);

            Point2D extrusion = new Point2D();
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, pointToExtrude);
            extrusions.add(extrusion);
         }
      }

      if (pointsToExtrude.size() >= 2)
      { // End
         int lastIndex = pointsToExtrude.size() - 1;
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(lastIndex);
         double extrusionDistance = extrusionDistances[lastIndex];

         Line2D edgePrev = new Line2D(pointsToExtrude.get(lastIndex - 1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, pointsToExtrude.get(lastIndex - 1));
         extrusions.addAll(extrudeCorner(pointToExtrude, edgePrev, edgeNext, true, numberOfExtrusionsAtEndpoints, extrusionDistance));
      }

      for (int i = pointsToExtrude.size() - 2; i >= 1; i--)
      { // Go from end back to start
         Point2DReadOnly pointToExtrude = pointsToExtrude.get(i);
         double extrusionDistance = extrusionDistances[i];

         Line2D edgePrev = new Line2D(pointsToExtrude.get(i + 1), pointToExtrude);
         Line2D edgeNext = new Line2D(pointToExtrude, pointsToExtrude.get(i - 1));

         boolean shouldExtrudeCorner = edgePrev.getDirection().angle(edgeNext.getDirection()) <= -HALF_PI;

         if (shouldExtrudeCorner)
         {
            extrusions.addAll(extrudeCorner(pointToExtrude, edgePrev, edgeNext, true, 3, extrusionDistance));
         }
         else
         {
            Vector2D extrusionDirection = new Vector2D();
            extrusionDirection.interpolate(edgePrev.getDirection(), edgeNext.getDirection(), 0.5);
            extrusionDirection.normalize();
            extrusionDirection = EuclidGeometryTools.perpendicularVector2D(extrusionDirection);

            Point2D extrusion = new Point2D();
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, pointToExtrude);
            extrusions.add(extrusion);
         }
      }

      if (!extrusions.isEmpty())
         extrusions.add(extrusions.get(0));

      return extrusions;
   }

   public static List<Point2D> extrudeLine(Point2DReadOnly endpoint1, Point2DReadOnly endpoint2, double extrusionDistance, int numberOfExtrusionsAtEndpoints)
   {
      return extrudeLine(endpoint1, extrusionDistance, endpoint2, extrusionDistance, numberOfExtrusionsAtEndpoints);
   }

   public static List<Point2D> extrudeLine(Point2DReadOnly endpoint1, double extrusionDistance1, Point2DReadOnly endpoint2, double extrusionDistance2,
                                           int numberOfExtrusionsAtEndpoints)
   {
      List<Point2D> extrusions = new ArrayList<>();
      Line2D edge1 = new Line2D(endpoint1, endpoint2);
      Line2D edge2 = new Line2D(endpoint2, endpoint1);

      List<Point2D> extrusions1 = extrudeCorner(endpoint1, edge2, edge1, true, numberOfExtrusionsAtEndpoints, extrusionDistance1);
      List<Point2D> extrusions2 = extrudeCorner(endpoint2, edge1, edge2, true, numberOfExtrusionsAtEndpoints, extrusionDistance2);
      extrusions.addAll(extrusions1);
      extrusions.addAll(extrusions2);
      extrusions.add(extrusions1.get(0));

      return extrusions;
   }

   public static List<Point2D> extrudeCorner(Point2DReadOnly cornerPointToExtrude, Line2D previousEdge, Line2D nextEdge, boolean extrudeToTheLeft,
                                             int numberOfExtrusions, double extrusionDistance)
   {
      List<Point2D> extrusions = new ArrayList<>();

      Vector2D firstExtrusionDirection = EuclidGeometryTools.perpendicularVector2D(previousEdge.getDirection());
      if (!extrudeToTheLeft)
         firstExtrusionDirection.negate();
      Point2D firstExtrusion = new Point2D();
      firstExtrusion.scaleAdd(extrusionDistance, firstExtrusionDirection, cornerPointToExtrude);
      extrusions.add(firstExtrusion);

      Vector2D lastExtrusionDirection = EuclidGeometryTools.perpendicularVector2D(nextEdge.getDirection());
      if (!extrudeToTheLeft)
         lastExtrusionDirection.negate();
      Point2D lastExtrusion = new Point2D();
      lastExtrusion.scaleAdd(extrusionDistance, lastExtrusionDirection, cornerPointToExtrude);

      if (numberOfExtrusions > 2)
      {
         double openingAngle = firstExtrusionDirection.angle(lastExtrusionDirection);
         if (MathTools.epsilonEquals(Math.PI, Math.abs(openingAngle), 1.0e-7))
            openingAngle = extrudeToTheLeft ? -Math.PI : Math.PI;

         Vector2D extrusionDirection = new Vector2D();

         for (int i = 1; i < numberOfExtrusions - 1; i++)
         {
            double alpha = i / (numberOfExtrusions - 1.0);
            RotationMatrixTools.applyYawRotation(alpha * openingAngle, firstExtrusionDirection, extrusionDirection);
            Point2D extrusion = new Point2D();
            extrusion.scaleAdd(extrusionDistance, extrusionDirection, cornerPointToExtrude);
            extrusions.add(extrusion);
         }
      }

      extrusions.add(lastExtrusion);

      return extrusions;
   }

   public static Cluster getTheClosestCluster(Point3DReadOnly pointToSortFrom, List<Cluster> clusters)
   {
      double minDistance = Double.MAX_VALUE;
      Cluster closestCluster = null;

      for (Cluster cluster : clusters)
      {
         double distOfPoint = Double.MAX_VALUE;
         Point3DReadOnly closestPointInCluster = null;

         for (Point3DReadOnly point : cluster.getNonNavigableExtrusionsInWorld())
         {
            double currentDistance = point.distanceSquared(pointToSortFrom);
            if (currentDistance < distOfPoint)
            {
               distOfPoint = currentDistance;
               closestPointInCluster = point;
            }
         }

         double currentDistance = closestPointInCluster.distanceSquared(pointToSortFrom);

         if (currentDistance < minDistance)
         {
            minDistance = currentDistance;
            closestCluster = cluster;
         }
      }

      return closestCluster;
   }

   public static FrameCluster getTheClosestFrameCluster(FramePoint3DReadOnly pointToSortFrom, List<FrameCluster> clusters)
   {
      double minDistance = Double.MAX_VALUE;
      FrameCluster closestCluster = null;

      for (FrameCluster cluster : clusters)
      {
         double distOfPoint = Double.MAX_VALUE;
         FramePoint2DReadOnly closestPointInCluster = null;

         for (FramePoint2DReadOnly point : cluster.getNonNavigableExtrusions())
         {
            double currentDistance = point.distanceXYSquared(pointToSortFrom);
            if (currentDistance < distOfPoint)
            {
               distOfPoint = currentDistance;
               closestPointInCluster = point;
            }
         }

         double currentDistance = closestPointInCluster.distanceXYSquared(pointToSortFrom);

         if (currentDistance < minDistance)
         {
            minDistance = currentDistance;
            closestCluster = cluster;
         }
      }

      return closestCluster;
   }

   public static Point3D getTheClosestVisibleExtrusionPoint(Point3DReadOnly pointToSortFrom, List<Point3D> extrusionPoints)
   {
      double minDistance = Double.MAX_VALUE;
      Point3D closestPoint = null;

      for (Point3D point : extrusionPoints)
      {
         double currentDistance = point.distanceSquared(pointToSortFrom);
         if (currentDistance < minDistance)
         {
            minDistance = currentDistance;
            closestPoint = point;
         }
      }

      return closestPoint;
   }

   public static Point3D getTheClosestVisibleExtrusionPoint(double alpha, Point3DReadOnly start, Point3DReadOnly goal,
                                                            List<? extends Point3DReadOnly> extrusionPoints, PlanarRegion region)
   {
      double minWeight = Double.MAX_VALUE;
      Point3DReadOnly closestPoint = null;

      for (Point3DReadOnly point : extrusionPoints)
      {
         if (PlanarRegionTools.isPointInWorldInsidePlanarRegion(region, point))
         {
            double weight = alpha * goal.distance(point) + (1 - alpha) * start.distance(point);

            if (weight < minWeight)
            {
               minWeight = weight;
               closestPoint = point;
            }
         }
      }

      return new Point3D(closestPoint);
   }

   public static FramePoint3DReadOnly getTheClosestVisibleExtrusionFramePoint(double alpha, FramePoint3DReadOnly start, FramePoint3DReadOnly goal,
                                                                              List<FramePoint2DBasics> extrusionPoints, PlanarRegion region)
   {
      double minWeight = Double.MAX_VALUE;
      FramePoint2DReadOnly closestPoint = null;

      for (FramePoint2DReadOnly point : extrusionPoints)
      {
         if (PlanarRegionTools.isPointInWorldInsidePlanarRegion(region, point))
         {
            double weight = alpha * goal.distanceXY(point) + (1 - alpha) * start.distanceXY(point);

            if (weight < minWeight)
            {
               minWeight = weight;
               closestPoint = point;
            }
         }
      }

      return new FramePoint3D(closestPoint);
   }

   public static Cluster createHomeRegionCluster(PlanarRegion homeRegion, NavigableExtrusionDistanceCalculator calculator)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      homeRegion.getTransformToWorld(transformToWorld);

      Cluster cluster = new Cluster();
      cluster.setType(Type.POLYGON);
      cluster.setTransformToWorld(transformToWorld);
      cluster.addRawPointsInLocal2D(homeRegion.getConcaveHull());
      cluster.setExtrusionSide(ExtrusionSide.INSIDE);

      double extrusionDistance = calculator.computeExtrusionDistance(homeRegion);

      ObstacleExtrusionDistanceCalculator nonNavigableCalculator = (p, h) -> extrusionDistance - NAV_TO_NON_NAV_DISTANCE;
      ObstacleExtrusionDistanceCalculator navigableCalculator = (p, h) -> extrusionDistance;

      boolean extrudeToTheLeft = cluster.getExtrusionSide() != ExtrusionSide.INSIDE;
      cluster.addNonNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, cluster, nonNavigableCalculator));
      cluster.addNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, cluster, navigableCalculator));
      cluster.updateBoundingBox();
      return cluster;
   }

   public static List<Cluster> createObstacleClusters(PlanarRegion homeRegion, List<PlanarRegion> obstacleRegions, double orthogonalAngle,
                                                      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator)
   {
      List<Cluster> obstacleClusters = new ArrayList<>();

      RigidBodyTransform transformFromHomeToWorld = new RigidBodyTransform();
      homeRegion.getTransformToWorld(transformFromHomeToWorld);
      Vector3D referenceNormal = homeRegion.getNormal();
      double zThresholdBeforeOrthogonal = Math.cos(orthogonalAngle);

      for (PlanarRegion obstacleRegion : obstacleRegions)
      {
         Vector3D otherNormal = obstacleRegion.getNormal();

         Cluster cluster = new Cluster();
         cluster.setExtrusionSide(ExtrusionSide.OUTSIDE);
         cluster.setTransformToWorld(transformFromHomeToWorld);

         List<Point3D> rawPointsInLocal = new ArrayList<>();
         RigidBodyTransform transformFromOtherToHome = new RigidBodyTransform();
         obstacleRegion.getTransformToWorld(transformFromOtherToHome);
         transformFromOtherToHome.preMultiplyInvertOther(transformFromHomeToWorld);

         for (int i = 0; i < obstacleRegion.getConvexHull().getNumberOfVertices(); i++)
         {
            Point3D concaveHullVertexHome = new Point3D(obstacleRegion.getConvexHull().getVertex(i));
            concaveHullVertexHome.applyTransform(transformFromOtherToHome);
            rawPointsInLocal.add(concaveHullVertexHome);
         }

         if (Math.abs(otherNormal.dot(referenceNormal)) < zThresholdBeforeOrthogonal)
         { // Project region as a line
            cluster.setType(Type.MULTI_LINE);
            cluster.addRawPointsInLocal3D(filterVerticalPolygonForMultiLineExtrusion(rawPointsInLocal, POPPING_MULTILINE_POINTS_THRESHOLD));
         }
         else
         { // Project region as a polygon
            cluster.setType(Type.POLYGON);
            cluster.addRawPointsInLocal3D(rawPointsInLocal);
         }

         extrudeObstacleCluster(cluster, extrusionDistanceCalculator);
         obstacleClusters.add(cluster);
      }

      return obstacleClusters;
   }

   public static void extrudeObstacleCluster(Cluster cluster, ObstacleExtrusionDistanceCalculator calculator)
   {
      ObstacleExtrusionDistanceCalculator nonNavigableCalculator = (p, h) -> calculator.computeExtrusionDistance(p, h) - NAV_TO_NON_NAV_DISTANCE;
      ObstacleExtrusionDistanceCalculator navigableCalculator = calculator;
      int numberOfExtrusionsAtEndpoints = 5;

      switch (cluster.getType())
      {
      case LINE:
      case MULTI_LINE:
         cluster.addNonNavigableExtrusionsInLocal(extrudeMultiLine(cluster, nonNavigableCalculator, numberOfExtrusionsAtEndpoints));
         cluster.addNavigableExtrusionsInLocal(extrudeMultiLine(cluster, navigableCalculator, numberOfExtrusionsAtEndpoints));
         break;
      case POLYGON:
         boolean extrudeToTheLeft = cluster.getExtrusionSide() != ExtrusionSide.INSIDE;

         try
         {
            cluster.addNonNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, cluster, nonNavigableCalculator));
         }
         catch (Exception e)
         {
            e.printStackTrace();
            return;
         }

         cluster.addNavigableExtrusionsInLocal(extrudePolygon(extrudeToTheLeft, cluster, navigableCalculator));
         break;

      default:
         throw new RuntimeException("Unhandled cluster type: " + cluster.getType());
      }
      cluster.updateBoundingBox();
   }

   /**
    *
    *
    * @param verticalPolygonVertices
    * @param poppingPointsDistanceSquaredThreshold
    * @return
    */
   static List<Point3D> filterVerticalPolygonForMultiLineExtrusion(List<? extends Point3DReadOnly> verticalPolygonVertices,
                                                                   double poppingPointsDistanceSquaredThreshold)
   {
      // Making a deep copy
      List<Point3D> filteredPoints = verticalPolygonVertices.stream().map(Point3D::new).collect(Collectors.toList());

      if (verticalPolygonVertices.size() <= 2)
         return filteredPoints;

      Point3D mean = new Point3D();
      Vector3D principalVector = new Vector3D();
      PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

      pca.clear();
      filteredPoints.forEach(p -> pca.addPoint(p.getX(), p.getY(), 0.0));
      pca.compute();
      pca.getMean(mean);
      pca.getPrincipalVector(principalVector);
      Line2D line = new Line2D(new Point2D(mean), new Vector2D(principalVector));
      // Projecting the points in the 2D plane described by the z-axis and the line direction
      List<Point2D> projectedPoints = new ArrayList<>();
      for (Point3D point : filteredPoints)
      {
         double x = line.parameterGivenPointOnLine(new Point2D(point), Double.POSITIVE_INFINITY);
         double z = point.getZ();
         projectedPoints.add(new Point2D(x, z));
      }

      for (int pointIndex = 0; pointIndex < projectedPoints.size(); pointIndex++)
      {
         Point3D point = filteredPoints.get(pointIndex);
         Point2D projectedPoint = projectedPoints.get(pointIndex);

         for (int edgeIndex = 0; edgeIndex < projectedPoints.size(); edgeIndex++)
         {
            Point2D edgeStart = projectedPoints.get(edgeIndex);
            Point2D edgeEnd = ListWrappingIndexTools.getNext(edgeIndex, projectedPoints);

            // Check if the point is between start and end
            double signedDistanceToStart = edgeStart.getX() - projectedPoint.getX();
            double signedDistanceToEnd = edgeEnd.getX() - projectedPoint.getX();
            if (signedDistanceToStart * signedDistanceToEnd > 0.0)
               continue; // If same sign, the edge is not above/below the point, keep going.
            // The edge is above or below the point, let's compute the edge height at the point x-coordinate
            double alpha = EuclidGeometryTools.percentageAlongLineSegment2D(projectedPoint, edgeStart, edgeEnd);
            double height = EuclidCoreTools.interpolate(edgeStart.getY(), edgeEnd.getY(), alpha);

            point.setZ(Math.max(height, point.getZ()));
         }

         // Adjust the XY-coordinates to be on the line
         point.set(line.pointOnLineGivenParameter(projectedPoint.getX()));
      }

      // Sort the points given their position on the line.
      Collections.sort(filteredPoints, (p1, p2) -> {
         double t1 = line.parameterGivenPointOnLine(new Point2D(p1), Double.POSITIVE_INFINITY);
         double t2 = line.parameterGivenPointOnLine(new Point2D(p2), Double.POSITIVE_INFINITY);
         return t1 >= t2 ? 1 : -1;
      });

      // FIXME Problem with the obstacle height.
      if (filteredPoints.get(0).distanceXYSquared(filteredPoints.get(filteredPoints.size() - 1)) <= poppingPointsDistanceSquaredThreshold)
      {
         double maxHeight = filteredPoints.stream().map(Point3D::getZ).max((d1, d2) -> Double.compare(d1, d2)).get();
         List<Point3D> endpoints = new ArrayList<>();
         endpoints.add(filteredPoints.get(0));
         endpoints.add(filteredPoints.get(filteredPoints.size() - 1));
         endpoints.forEach(p -> p.setZ(maxHeight));
         return endpoints;
      }
      else
      {
         int index = 0;
         // Look for points with same XY-coordinates and only keep the highest one.
         while (index < filteredPoints.size() - 1)
         {
            Point3D pointCurr = filteredPoints.get(index);
            Point3D pointNext = filteredPoints.get(index + 1);

            if (pointCurr.distanceXYSquared(pointNext) < poppingPointsDistanceSquaredThreshold)
               filteredPoints.remove(pointCurr.getZ() <= pointNext.getZ() ? index : index + 1);
            else
               index++;
         }
         return filteredPoints;
      }
   }
}

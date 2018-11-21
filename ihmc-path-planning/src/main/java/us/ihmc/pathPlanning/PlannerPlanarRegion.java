package us.ihmc.pathPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.shapes.FramePlane3d;
import us.ihmc.robotics.random.RandomGeometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class PlannerPlanarRegion
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final int NO_REGION_ID = -1;
   public static final double DEFAULT_BOUNDING_BOX_EPSILON = 0.0;

   private int regionId = NO_REGION_ID;

   private ReferenceFrame referenceFrame;

   private final FramePoint2D[] concaveHullsVertices;

   private final List<FrameConvexPolygon2D> convexPolygons;
   private final FrameConvexPolygon2D convexHull = new FrameConvexPolygon2D();

   private final BoundingBox3D boundingBox3dInWorld = new BoundingBox3D(new Point3D(Double.NaN, Double.NaN, Double.NaN),
                                                                        new Point3D(Double.NaN, Double.NaN, Double.NaN));
   private double boundingBoxEpsilon = DEFAULT_BOUNDING_BOX_EPSILON;


   public PlannerPlanarRegion()
   {
      referenceFrame = worldFrame;
      concaveHullsVertices = new FramePoint2D[0];
      convexPolygons = new ArrayList<>();
      updateConvexHull();
      updateBoundingBox();
   }

   public PlannerPlanarRegion(PlanarRegion planarRegion)
   {
      referenceFrame = new ReferenceFrame("plannerPlanarRegion", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            planarRegion.getTransformToWorld(transformToParent);
         }
      };
      referenceFrame.update();
      concaveHullsVertices = new FramePoint2D[planarRegion.getConcaveHullSize()];
      for (int i = 0; i < planarRegion.getConcaveHullSize(); i++)
         concaveHullsVertices[i] = new FramePoint2D(referenceFrame, planarRegion.getConcaveHullVertex(i));
      convexPolygons = new ArrayList<>();
      for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
         convexPolygons.add(new FrameConvexPolygon2D(referenceFrame, planarRegion.getConvexPolygon(i)));
      updateConvexHull();
      updateBoundingBox();
   }

   /**
    * Create a new planar region.
    *
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param planarRegionConvexPolygons the list of convex polygon that represents the planar
    *           region. Expressed in local coordinate system.
    */
   public PlannerPlanarRegion(RigidBodyTransform transformToWorld, List<FrameConvexPolygon2D> planarRegionConvexPolygons)
   {
      PrintTools.warn("This constructor does not set the concave hull.");

      referenceFrame = new ReferenceFrame("plannerPlanarRegion", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformToWorld);
         }
      };
      referenceFrame.update();
      concaveHullsVertices = new FramePoint2D[0];
      convexPolygons = planarRegionConvexPolygons;
      updateConvexHull();
      updateBoundingBox();
   }

   /**
    * Create a new planar region.
    *
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param concaveHullVertices vertices of the concave hull of the region.
    * @param planarRegionConvexPolygons the list of convex polygon that represents the planar
    *           region. Expressed in local coordinate system.
    */
   public PlannerPlanarRegion(RigidBodyTransform transformToWorld, FramePoint2D[] concaveHullVertices, List<FrameConvexPolygon2D> planarRegionConvexPolygons)
   {
      referenceFrame = new ReferenceFrame("plannerPlanarRegion", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformToWorld);
         }
      };
      referenceFrame.update();
      this.concaveHullsVertices = concaveHullVertices;
      convexPolygons = planarRegionConvexPolygons;
      updateConvexHull();
      updateBoundingBox();
   }

   public PlannerPlanarRegion(ReferenceFrame referenceFrame, FramePoint2D[] concaveHullVertices, List<FrameConvexPolygon2D> planarRegionConvexPolygons)
   {
      this.referenceFrame = referenceFrame;
      this.concaveHullsVertices = concaveHullVertices;
      convexPolygons = planarRegionConvexPolygons;
      updateConvexHull();
      updateBoundingBox();
   }

   /**
    * Create a new planar region.
    *
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param convexPolygon a single convex polygon that represents the planar region. Expressed in
    *           local coordinate system.
    */
   public PlannerPlanarRegion(RigidBodyTransform transformToWorld, FrameConvexPolygon2D convexPolygon)
   {
      referenceFrame = new ReferenceFrame("plannerPlanarRegion", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformToWorld);
         }
      };
      referenceFrame.update();
      concaveHullsVertices = new FramePoint2D[convexPolygon.getNumberOfVertices()];
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         concaveHullsVertices[i] = new FramePoint2D(convexPolygon.getVertex(i));
      }
      convexPolygons = new ArrayList<>();
      convexPolygons.add(convexPolygon);
      updateConvexHull();
      updateBoundingBox();
   }

   public void set(RigidBodyTransform transformToWorld, List<FrameConvexPolygon2D> planarRegionConvexPolygons)
   {
      this.set(transformToWorld, planarRegionConvexPolygons, NO_REGION_ID);
   }

   public void set(RigidBodyTransform transformToWorld, List<FrameConvexPolygon2D> planarRegionConvexPolygons, int newRegionId)
   {
      referenceFrame = new ReferenceFrame("plannerPlanarRegion", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformToWorld);
         }
      };
      referenceFrame.update();

      convexPolygons.clear();
      convexPolygons.addAll(planarRegionConvexPolygons);

      updateConvexHull();
      updateBoundingBox();

      regionId = newRegionId;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Check if the given lineSegment intersects this region projected onto the XY-plane.
    *
    * @param lineSegmentInWorld
    * @return true if the lineSegment intersects this PlanarRegion.
    */
   public boolean isLineSegmentIntersecting(FrameLineSegment2DReadOnly lineSegmentInWorld)
   {
      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given lineSegment is projected along the z-world axis to be snapped onto plane.
      FrameLineSegment2DReadOnly projectedLineSegment = projectLineSegmentVerticallyToRegion(lineSegmentInWorld);

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         FrameConvexPolygon2D polygonToCheck = convexPolygons.get(i);
         FramePoint2DBasics[] intersectionPoints = polygonToCheck.intersectionWith(projectedLineSegment);
         if ((intersectionPoints != null) && (intersectionPoints.length > 0) && (intersectionPoints[0] != null))
            return true;
      }
      // Did not find any intersection
      return false;
   }

   /**
    * Returns all of the intersections when the convexPolygon is projected vertically onto this
    * PlanarRegion.
    *
    * @param intersectionsInPlaneFrameToPack ArrayList of ConvexPolygon2d to pack with the
    *           intersections.
    */
   public void getLineSegmentIntersectionsWhenProjectedVertically(FrameLineSegment2DReadOnly lineSegmentInWorld, List<FramePoint2DBasics[]> intersectionsInPlaneFrameToPack)
   {
      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given lineSegment is projected along the z-world axis to be snapped onto plane.
      FrameLineSegment2DReadOnly projectedLineSegment = projectLineSegmentVerticallyToRegion(lineSegmentInWorld);

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         FramePoint2DBasics[] intersectionPoints = convexPolygons.get(i).intersectionWith(projectedLineSegment);

         if ((intersectionPoints != null) && (intersectionPoints.length > 0) && (intersectionPoints[0] != null))
         {
            intersectionsInPlaneFrameToPack.add(intersectionPoints);
         }
      }
   }




   /**
    * Projects the input ConvexPolygon2d to the plane defined by this PlanarRegion by translating
    * each vertex in world z. Then puts each vertex in local frame. In doing so, the area of the
    * rotated polygon will actually increase on tilted PlanarRegions.
    *
    * @param convexPolygon Polygon to project
    * @return new projected ConvexPolygon2d
    */
   private FrameConvexPolygon2DReadOnly projectPolygonVerticallyToRegion(FrameConvexPolygon2DReadOnly convexPolygon)
   {
      convexPolygon.checkReferenceFrameMatch(worldFrame);
      FrameConvexPolygon2D projectedPolygon = new FrameConvexPolygon2D(referenceFrame);

      FramePoint3D snappedVertex3d = new FramePoint3D();

      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         FramePoint2DReadOnly originalVertex = convexPolygon.getVertex(i);
         // Find the vertex 3d that is snapped to the plane following z-world.
         snappedVertex3d.setIncludingFrame(worldFrame, originalVertex.getX(), originalVertex.getY(), getPlaneZInWorldGivenXY(originalVertex));

         // Add the snapped vertex to the snapped polygon
         projectedPolygon.addVertexMatchingFrame(referenceFrame, snappedVertex3d);
      }
      projectedPolygon.update();
      return projectedPolygon;
   }

   /**
    * Projects the input LineSegment2d to the plane defined by this PlanarRegion by translating each
    * vertex in world z. Then puts each vertex in local frame. In doing so, the length of the
    * rotated lineSegment will actually increase on tilted PlanarRegions.
    *
    * @param lineSegmentInWorld LineSegment2d to project
    * @return new projected LineSegment2d
    */
   private FrameLineSegment2DReadOnly projectLineSegmentVerticallyToRegion(FrameLineSegment2DReadOnly lineSegmentInWorld)
   {
      lineSegmentInWorld.checkReferenceFrameMatch(worldFrame);
      FramePoint2DReadOnly originalVertex = lineSegmentInWorld.getFirstEndpoint();
      FramePoint3D snappedVertex3d = new FramePoint3D(worldFrame);

      // Find the vertex 3d that is snapped to the plane following z-world.
      snappedVertex3d.setIncludingFrame(worldFrame, originalVertex.getX(), originalVertex.getY(),
                                        getPlaneZInWorldGivenXY(originalVertex.getX(), originalVertex.getY()));

      // Transform to local coordinates
      snappedVertex3d.changeFrame(referenceFrame);
      FramePoint2D snappedFirstEndpoint = new FramePoint2D(snappedVertex3d);

      originalVertex = lineSegmentInWorld.getSecondEndpoint();

      // Find the vertex 3d that is snapped to the plane following z-world.
      snappedVertex3d.setIncludingFrame(worldFrame, originalVertex.getX(), originalVertex.getY(),
                                        getPlaneZInWorldGivenXY(originalVertex.getX(), originalVertex.getY()));

      // Transform to local coordinates
      snappedVertex3d.changeFrame(referenceFrame);
      FramePoint2D snappedSecondEndpoint = new FramePoint2D(snappedVertex3d);

      return new FrameLineSegment2D(snappedFirstEndpoint, snappedSecondEndpoint);
   }

   /**
    * Computes if the point is in the region projected onto the world xy-plane. Note that the
    * z-coordinate of the query is ignored.
    *
    * @param point3d query coordinates.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInsideByProjectionOntoXYPlane(FramePoint3DReadOnly point3d)
   {
      return isPointInsideByProjectionOntoXYPlane(point3d.getReferenceFrame(), point3d.getX(), point3d.getY());
   }

   /**
    * Computes if the point is in the region projected onto the world xy-plane.
    *
    * @param point2d query coordinates.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInsideByProjectionOntoXYPlane(FramePoint2DReadOnly point2d)
   {
      return isPointInsideByProjectionOntoXYPlane(point2d.getReferenceFrame(), point2d.getX(), point2d.getY());
   }

   /**
    * Computes if the point is in the region projected onto the world xy-plane.
    *
    * @param x x-coordinate of the query.
    * @param y y-coordinate of the query.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInsideByProjectionOntoXYPlane(ReferenceFrame referenceFrame, double x, double y)
   {
      referenceFrame.checkReferenceFrameMatch(worldFrame);
      FramePoint3D localPoint = new FramePoint3D(worldFrame, x, y, getPlaneZInWorldGivenXY(x, y));
      localPoint.changeFrame(referenceFrame);

      return isPointInside(localPoint);
   }

   /**
    * Computes the distance of the point to the region projected onto the world xy-plane.
    *
    * @param point2d query coordinates.
    * @return distance to this region. If 0.0, point is in the region.
    */
   public double distanceToPointByProjectionOntoXYPlane(FramePoint2DReadOnly point2d)
   {
      return distanceToPointByProjectionOntoXYPlane(point2d.getReferenceFrame(), point2d.getX(), point2d.getY());
   }

   private final FramePoint3D localPoint = new FramePoint3D();
   private final FramePoint2D localPoint2D = new FramePoint2D();
   /**
    * Computes the distance of the point to the region projected onto the world xy-plane.
    *
    * @param x x-coordinate of the query.
    * @param y y-coordinate of the query.
    * @return distance to this region. If 0.0, point is in the region.
    */
   public double distanceToPointByProjectionOntoXYPlane(ReferenceFrame referenceFrame, double x, double y)
   {
      referenceFrame.checkReferenceFrameMatch(worldFrame);

      localPoint.setIncludingFrame(worldFrame, x, y, getPlaneZInWorldGivenXY(x, y));
      localPoint.changeFrame(referenceFrame);

      localPoint2D.setIncludingFrame(localPoint);

      return distanceToPoint(localPoint2D);
   }

   /**
    * Given a 3D point in world coordinates, computes whether the point is in this region.
    *
    * @param point3dInWorld query expressed in world coordinates.
    * @param maximumOrthogonalDistance tolerance expressed as maximum orthogonal distance from the
    *           region.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(FramePoint3DReadOnly point3dInWorld, double maximumOrthogonalDistance)
   {
      FramePoint3D localPoint = new FramePoint3D(point3dInWorld);
      localPoint.changeFrame(referenceFrame);

      if (!MathTools.intervalContains(localPoint.getZ(), maximumOrthogonalDistance))
         return false;
      else
         return isPointInside(localPoint);
   }


   /**
    * Checks to see if a given point is on the plane or above it by the specified distance.
    *
    * @param point3dInWorld the point to check
    * @return True if the point is on the plane or no more than distanceFromPlane above it.
    */
   public boolean isPointInWorld2DInside(FramePoint3DReadOnly point3dInWorld)
   {
      FramePoint2D localPoint = new FramePoint2D(point3dInWorld);
      localPoint.changeFrame(referenceFrame);

      boolean isInsideXY = isPointInside(localPoint);

      return isInsideXY;
   }

   /**
    * Checks to see if a given point is on the plane or above it by the specified distance.
    *
    * @param point the point to check
    * @param distanceFromPlane The distance above the plane that the point is allowed to be
    * @return True if the point is on the plane or no more than distanceFromPlane above it.
    */
   public boolean isPointOnOrSlightlyAbove(FramePoint3DReadOnly point, double distanceFromPlane)
   {
      MathTools.checkPositive(distanceFromPlane);
      FramePoint3D localPoint = new FramePoint3D(point);
      localPoint.changeFrame(referenceFrame);

      boolean onOrAbove = localPoint.getZ() >= 0.0;
      boolean withinDistance = localPoint.getZ() < distanceFromPlane;
      boolean isInsideXY = isPointInside(localPoint);

      return onOrAbove && withinDistance && isInsideXY;
   }

   /**
    * Checks to see if a given point is on the plane or below it by the specified distance.
    *
    * @param point the point to check
    * @param distanceFromPlane The distance below the plane that the point is allowed to be
    * @return True if the point is on the plane or no more than distanceFromPlane below it.
    */
   public boolean isPointOnOrSlightlyBelow(FramePoint3DReadOnly point, double distanceFromPlane)
   {
      MathTools.checkPositive(distanceFromPlane);
      FramePoint3D localPoint = new FramePoint3D(point);
      localPoint.changeFrame(referenceFrame);

      boolean onOrBelow = localPoint.getZ() <= 0.0;
      boolean withinDistance = localPoint.getZ() > (distanceFromPlane * -1.0);
      boolean isInsideXY = isPointInside(localPoint);

      return onOrBelow && withinDistance && isInsideXY;
   }

   /**
    * Given a 2D point expressed in the plane local frame, computes whether the point is in this
    * region.
    *
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(FramePoint2DReadOnly point)
   {
      for (int i = 0; i < convexPolygons.size(); i++)
      {
         if (convexPolygons.get(i).isPointInside(point))
            return true;
      }
      return false;
   }

   /**
    * Given a 2D point expressed in the plane local frame, computes whether the point is in this
    * region.
    *
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(FramePoint3DReadOnly point)
   {
      for (int i = 0; i < convexPolygons.size(); i++)
      {
         if (convexPolygons.get(i).isPointInside(new FramePoint2D(point)))
            return true;
      }
      return false;
   }

   /**
    * Given a 2D point expressed in the plane local frame, computes whether the point is in this
    * region.
    *
    * @param localPoint Coordinate of the 2D point in planar region local frame
    * @return shortest distance from the point to the planar region
    */
   public double distanceToPoint(FramePoint2DReadOnly localPoint)
   {
      localPoint.checkReferenceFrameMatch(referenceFrame);
      double shortestDistanceToPoint = Double.POSITIVE_INFINITY;
      for (int i = 0; i < convexPolygons.size(); i++)
      {
         double distance = convexPolygons.get(i).distance(localPoint);
         if (distance < shortestDistanceToPoint)
            shortestDistanceToPoint = distance;
      }
      return shortestDistanceToPoint;
   }

   /**
    * Computes the z-coordinate in world of the plane for a given xy-coordinates in world.
    *
    * @return the z-coordinate
    */
   public double getPlaneZInWorldGivenXY(FramePoint2DReadOnly point)
   {
      point.checkReferenceFrameMatch(referenceFrame);
      return getPlaneZInWorldGivenXY(point.getX(), point.getY());
   }

   /**
    * Computes the z-coordinate in world of the plane for a given xy-coordinates in world.
    *
    * @param xWorld x-coordinate of the query
    * @param yWorld y-coordinate of the query
    * @return the z-coordinate
    */
   public double getPlaneZInWorldGivenXY(double xWorld, double yWorld)
   {
      RigidBodyTransform fromLocalToWorldTransform = referenceFrame.getTransformToWorldFrame();
      // The three components of the plane origin
      double x0 = fromLocalToWorldTransform.getM03();
      double y0 = fromLocalToWorldTransform.getM13();
      double z0 = fromLocalToWorldTransform.getM23();
      // The three components of the plane normal
      double a = fromLocalToWorldTransform.getM02();
      double b = fromLocalToWorldTransform.getM12();
      double c = fromLocalToWorldTransform.getM22();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - xWorld) + b / c * (y0 - yWorld) + z0;
      return z;
   }

   /**
    * Every can be given a unique. The default value is {@value #NO_REGION_ID} which corresponds to
    * no id.
    *
    * @param regionId set the unique id of this region.
    */
   public void setRegionId(int regionId)
   {
      this.regionId = regionId;
   }

   /**
    * @return the unique id of this regions. It is equal to {@value #NO_REGION_ID} when no id has
    *         been attributed.
    */
   public int getRegionId()
   {
      return regionId;
   }

   /**
    * @return whether a unique id has been attributed to this region or not.
    */
   public boolean hasARegionId()
   {
      return regionId != NO_REGION_ID;
   }

   /**
    * Returns true only if there is no polygons in this planar region. Does not check for empty
    * polygons.
    */
   public boolean isEmpty()
   {
      return convexPolygons.isEmpty();
   }

   public FramePoint2D[] getConcaveHull()
   {
      return concaveHullsVertices;
   }

   public FramePoint2D getConcaveHullVertex(int i)
   {
      return concaveHullsVertices[i];
   }

   public int getConcaveHullSize()
   {
      return concaveHullsVertices.length;
   }

   /** Returns the number of convex polygons representing this region. */
   public int getNumberOfConvexPolygons()
   {
      return convexPolygons.size();
   }

   public List<FrameConvexPolygon2D> getConvexPolygons()
   {
      return convexPolygons;
   }

   /**
    * Returns the i<sup>th</sup> convex polygon representing a portion of this region. The polygon
    * is expressed in the region local coordinates.
    */
   public FrameConvexPolygon2D getConvexPolygon(int i)
   {
      return convexPolygons.get(i);
   }

   /**
    * Returns the last convex polygon representing a portion of this region. Special case: returns
    * null when this region is empty. The polygon is expressed in the region local coordinates.
    */
   public FrameConvexPolygon2D getLastConvexPolygon()
   {
      if (isEmpty())
         return null;
      else
         return getConvexPolygon(getNumberOfConvexPolygons() - 1);
   }

   /**
    * Returns the i<sup>th</sup> convex polygon representing a portion of this region and removes it
    * from this planar region. The polygon is expressed in the region local coordinates.
    */
   public FrameConvexPolygon2D pollConvexPolygon(int i)
   {
      FrameConvexPolygon2D polledPolygon = convexPolygons.remove(i);
      updateConvexHull();
      updateBoundingBox();
      return polledPolygon;
   }

   /**
    * Returns the last convex polygon representing a portion of this region and removes it from this
    * planar region. Special case: returns null when this region is empty. The polygon is expressed
    * in the region local coordinates.
    */
   public FrameConvexPolygon2D pollLastConvexPolygon()
   {
      if (isEmpty())
         return null;
      else
         return pollConvexPolygon(getNumberOfConvexPolygons() - 1);
   }

   /**
    * Retrieves and returns a copy of the normal in world frame of this planar region.
    */
   public FrameVector3DReadOnly getNormal()
   {
      FrameVector3D normal = new FrameVector3D();
      getNormal(normal);
      return normal;
   }

   /**
    * Retrieves the normal of this planar region in the world frame and stores it in the given {@link Vector3D}.
    *
    * @param normalToPack used to store the normal of this planar region.
    */
   public void getNormal(FrameVector3DBasics normalToPack)
   {
      RigidBodyTransform fromLocalToWorldTransform = referenceFrame.getTransformToWorldFrame();
      normalToPack.setToZero(worldFrame);
      normalToPack.setX(fromLocalToWorldTransform.getM02());
      normalToPack.setY(fromLocalToWorldTransform.getM12());
      normalToPack.setZ(fromLocalToWorldTransform.getM22());
   }

   /**
    * Returns true if this PlanarRegion is purely vertical, as far as numerical roundoff is
    * concerned. Checks z component of surface normal. If absolute value is really small, then
    * returns true.
    *
    * @return true if vertical. false otherwise.
    */
   public boolean isVertical()
   {
      return (Math.abs(referenceFrame.getTransformToWorldFrame().getM22()) < 1e-10);
   }

   /**
    * Retrieves a point that lies in this planar region. This point is also used as the origin of
    * the local coordinate system of this planar region.
    *
    * @param pointToPack used to store the point coordinates.
    */
   public void getPointInRegion(FramePoint3DBasics pointToPack)
   {
      pointToPack.setToZero(referenceFrame);
   }


   public boolean epsilonEquals(PlannerPlanarRegion other, double epsilon)
   {
      if (!referenceFrame.equals(other.referenceFrame))
         return false;

      if (getNumberOfConvexPolygons() != other.getNumberOfConvexPolygons())
         return false;

      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         if (!convexPolygons.get(i).epsilonEquals(other.convexPolygons.get(i), epsilon))
            return false;
      }
      return true;
   }

   public void set(PlannerPlanarRegion other)
   {
      referenceFrame = other.referenceFrame;
      convexPolygons.clear();
      for (int i = 0; i < other.getNumberOfConvexPolygons(); i++)
         convexPolygons.add(new FrameConvexPolygon2D(other.convexPolygons.get(i)));

      convexHull.set(other.convexHull);
   }


   private void updateConvexHull()
   {
      convexHull.clear();
      for (int i = 0; i < this.getNumberOfConvexPolygons(); i++)
      {
         FrameConvexPolygon2DReadOnly convexPolygon = this.getConvexPolygon(i);
         for (int j = 0; j < convexPolygon.getNumberOfVertices(); j++)
            convexHull.addVertex(convexPolygon.getVertex(j));
      }
      convexHull.update();
   }

   private final FramePoint3D tempPointForConvexPolygonProjection = new FramePoint3D();

   private void updateBoundingBox()
   {
      boundingBox3dInWorld.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      for (int i = 0; i < this.getNumberOfConvexPolygons(); i++)
      {
         FrameConvexPolygon2D convexPolygon = this.getConvexPolygon(i);

         for (int j = 0; j < convexPolygon.getNumberOfVertices(); j++)
         {
            FramePoint2DReadOnly vertex = convexPolygon.getVertex(j);
            tempPointForConvexPolygonProjection.setIncludingFrame(vertex.getReferenceFrame(), vertex.getX(), vertex.getY(), 0.0);
            tempPointForConvexPolygonProjection.changeFrame(worldFrame);

            this.boundingBox3dInWorld.updateToIncludePoint(tempPointForConvexPolygonProjection);
         }
      }

      Point3DReadOnly minPoint = boundingBox3dInWorld.getMinPoint();
      Point3DReadOnly maxPoint = boundingBox3dInWorld.getMaxPoint();

      this.boundingBox3dInWorld.setMin(minPoint.getX() - boundingBoxEpsilon, minPoint.getY() - boundingBoxEpsilon, minPoint.getZ() - boundingBoxEpsilon);
      this.boundingBox3dInWorld.setMax(maxPoint.getX() + boundingBoxEpsilon, maxPoint.getY() + boundingBoxEpsilon, maxPoint.getZ() + boundingBoxEpsilon);
   }

   /**
    * @return a full depth copy of this region. The copy can be entirely modified without
    *         interfering with this region.
    */
   public PlannerPlanarRegion copy()
   {
      FramePoint2D[] concaveHullCopy = new FramePoint2D[concaveHullsVertices.length];
      for (int i = 0; i < concaveHullsVertices.length; i++)
         concaveHullCopy[i] = new FramePoint2D(concaveHullsVertices[i]);

      List<FrameConvexPolygon2D> convexPolygonsCopy = new ArrayList<>();
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
         convexPolygonsCopy.add(new FrameConvexPolygon2D(convexPolygons.get(i)));

      PlannerPlanarRegion planarRegion = new PlannerPlanarRegion(referenceFrame, concaveHullCopy, convexPolygonsCopy);
      planarRegion.setRegionId(regionId);
      return planarRegion;
   }

   /**
    * @return the convex hull of the region.
    */
   public FrameConvexPolygon2D getConvexHull()
   {
      return convexHull;
   }

   public static PlannerPlanarRegion generatePlanarRegionFromRandomPolygonsWithRandomTransform(Random random, int numberOfRandomlyGeneratedPolygons,
                                                                                        double maxAbsoluteXYForPolygons, int numberOfPossiblePointsForPolygons)
   {
      List<FrameConvexPolygon2D> regionConvexPolygons = new ArrayList<>();

      for (int i = 0; i < numberOfRandomlyGeneratedPolygons; i++)
      {
         FrameConvexPolygon2D randomPolygon = EuclidFrameRandomTools.nextFrameConvexPolygon2D(random, worldFrame, maxAbsoluteXYForPolygons, numberOfPossiblePointsForPolygons);
         regionConvexPolygons.add(randomPolygon);
      }

      for (FrameConvexPolygon2D convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      Vector3D randomTranslation = RandomGeometry.nextVector3D(random, 10.0);
      Quaternion randomOrientation = RandomGeometry.nextQuaternion(random, Math.toRadians(45.0));
      RigidBodyTransform regionTransform = new RigidBodyTransform(randomOrientation, randomTranslation);

      return new PlannerPlanarRegion(regionTransform, regionConvexPolygons);
   }


   public void update()
   {
      updateConvexHull();
      updateBoundingBox();
   }


   /**
    * Creates and returns the Plane3D that this planar region lies on.
    */
   public FramePlane3d getPlane()
   {
      FramePlane3d ret = new FramePlane3d(referenceFrame);
      ret.changeFrame(worldFrame);
      return ret;
   }

   public BoundingBox3D getBoundingBox3dInWorld()
   {
      return this.boundingBox3dInWorld;
   }
}

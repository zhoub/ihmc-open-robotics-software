package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class FrameConnectionPoint3DReadOnly implements FramePoint3DReadOnly
{
   public static final double PRECISION     = 1.0e-4;
   public static final double INV_PRECISION = 1.0e+4;

   protected final int regionId;
   protected final int hashCode;

   protected double x, y, z;
   protected ReferenceFrame referenceFrame;

   public FrameConnectionPoint3DReadOnly(FrameConnectionPoint3DReadOnly other)
   {
      this(other, other.regionId, other.referenceFrame);
   }

   public FrameConnectionPoint3DReadOnly(int regionId, ReferenceFrame referenceFrame)
   {
      this(0.0, 0.0, 0.0, regionId, referenceFrame);
   }

   public FrameConnectionPoint3DReadOnly(Tuple2DReadOnly other, int regionId, ReferenceFrame referenceFrame)
   {
      this(other.getX(), other.getY(), 0.0, regionId, referenceFrame);
   }

   public FrameConnectionPoint3DReadOnly(Tuple3DReadOnly other, int regionId, ReferenceFrame referenceFrame)
   {
      this(other.getX(), other.getY(), other.getZ(), regionId, referenceFrame);
   }

   public FrameConnectionPoint3DReadOnly(FrameTuple2DReadOnly other, int regionId)
   {
      this(other.getX(), other.getY(), 0.0, regionId, other.getReferenceFrame());
   }

   public FrameConnectionPoint3DReadOnly(FrameTuple3DReadOnly other, int regionId)
   {
      this(other.getX(), other.getY(), other.getZ(), regionId, other.getReferenceFrame());
   }

   public FrameConnectionPoint3DReadOnly(double x, double y, double z, int regionId, ReferenceFrame referenceFrame)
   {
      this.regionId = regionId;
      this.referenceFrame = referenceFrame;
      this.x = x;
      this.y = y;
      this.z = z;
      hashCode = computeHashCode();
   }

   private int computeHashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(getRoundedX());
      bits = 31L * bits + Double.doubleToLongBits(getRoundedY());
      bits = 31L * bits + Double.doubleToLongBits(getRoundedZ());
      return (int) (bits ^ bits >> 32);
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

   @Override
   public double getZ()
   {
      return z;
   }

   public int getRegionId()
   {
      return regionId;
   }

   public double getRoundedX()
   {
      return round(x);
   }

   public double getRoundedY()
   {
      return round(y);
   }

   public double getRoundedZ()
   {
      return round(z);
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == null)
         return false;

      try
      {
         return epsilonEquals((Tuple3DReadOnly) obj, PRECISION);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   @Override
   public boolean equals(Tuple3DReadOnly other)
   {
      return epsilonEquals(other, PRECISION);
   }

   @Override
   public String toString()
   {
      return "ConnectionPoint3D: " + EuclidCoreIOTools.getTuple3DString(this);
   }

   static double round(double value)
   {
      return Math.round(value * INV_PRECISION) * PRECISION;
   }


   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}

package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class FrameConnectionPoint3D extends FrameConnectionPoint3DReadOnly implements FramePoint3DBasics
{
   public FrameConnectionPoint3D(FrameConnectionPoint3DReadOnly other)
   {
      super(other);
   }

   public FrameConnectionPoint3D(int regionId, ReferenceFrame referenceFrame)
   {
      super(regionId, referenceFrame);
   }

   public FrameConnectionPoint3D(Tuple2DReadOnly other, int regionId, ReferenceFrame referenceFrame)
   {
      super(other, regionId, referenceFrame);
   }

   public FrameConnectionPoint3D(Tuple3DReadOnly other, int regionId, ReferenceFrame referenceFrame)
   {
      super(other, regionId, referenceFrame);
   }

   public FrameConnectionPoint3D(FrameTuple2DReadOnly other, int regionId)
   {
      super(other, regionId);
   }

   public FrameConnectionPoint3D(FrameTuple3DReadOnly other, int regionId)
   {
      super(other, regionId);
   }

   public FrameConnectionPoint3D(double x, double y, double z, int regionId, ReferenceFrame referenceFrame)
   {
      super(x, y, z, regionId, referenceFrame);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   @Override
   public void setZ(double z)
   {
      this.z = z;
   }
}

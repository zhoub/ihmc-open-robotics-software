package us.ihmc.robotics.math.frames;

import gnu.trove.impl.Constants;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class FrameYoFramePoint3D implements FramePoint3DBasics
{
   /** Represents null in the frame map since the reference frame ID can not be negative. */
   private static final long NO_ENTRY_KEY = -1L;
   private final TLongObjectMap<ReferenceFrame> frameMap = new TLongObjectHashMap<>(Constants.DEFAULT_CAPACITY, Constants.DEFAULT_LOAD_FACTOR, NO_ENTRY_KEY);

   private final YoDouble x;
   private final YoDouble y;
   private final YoDouble z;
   private final YoLong frameId;

   public FrameYoFramePoint3D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      z = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
      frameId = new YoLong(YoFrameVariableNameTools.createName(namePrefix, "FrameID", nameSuffix), registry);
      frameId.set(NO_ENTRY_KEY);
   }

   @Override
   public double getX()
   {
      return x.getValue();
   }

   @Override
   public double getY()
   {
      return y.getValue();
   }

   @Override
   public double getZ()
   {
      return z.getValue();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      if (frameId.getValue() == NO_ENTRY_KEY)
      {
         return null;
      }
      return frameMap.get(frameId.getValue());
   }

   @Override
   public void setX(double x)
   {
      this.x.set(x);
   }

   @Override
   public void setY(double y)
   {
      this.y.set(y);
   }

   @Override
   public void setZ(double z)
   {
      this.z.set(z);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      if (referenceFrame == null)
      {
         this.frameId.set(NO_ENTRY_KEY);
         return;
      }

      long frameId = referenceFrame.getNameBasedHashCode();
      if (frameId == NO_ENTRY_KEY)
      {
         throw new RuntimeException("The hash of a reference frame should never be " + NO_ENTRY_KEY);
      }

      if (!frameMap.containsKey(frameId))
      {
         frameMap.put(frameId, referenceFrame);
      }

      this.frameId.set(frameId);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      FramePoint3DBasics.super.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      FramePoint3DBasics.super.applyInverseTransform(transform);
   }

}

package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC quadruped controller API.
       * This message sends a desired body pose to the body teleop module.
       */
public class QuadrupedTeleopDesiredPose extends Packet<QuadrupedTeleopDesiredPose> implements Settable<QuadrupedTeleopDesiredPose>, EpsilonComparable<QuadrupedTeleopDesiredPose>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public us.ihmc.euclid.geometry.Pose3D pose_;
   public double pose_shift_time_;

   public QuadrupedTeleopDesiredPose()
   {
      pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public QuadrupedTeleopDesiredPose(QuadrupedTeleopDesiredPose other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedTeleopDesiredPose other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.pose_, pose_);
      pose_shift_time_ = other.pose_shift_time_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   public us.ihmc.euclid.geometry.Pose3D getPose()
   {
      return pose_;
   }

   public void setPoseShiftTime(double pose_shift_time)
   {
      pose_shift_time_ = pose_shift_time;
   }
   public double getPoseShiftTime()
   {
      return pose_shift_time_;
   }


   public static Supplier<QuadrupedTeleopDesiredPosePubSubType> getPubSubType()
   {
      return QuadrupedTeleopDesiredPosePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedTeleopDesiredPosePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedTeleopDesiredPose other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.pose_.epsilonEquals(other.pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pose_shift_time_, other.pose_shift_time_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedTeleopDesiredPose)) return false;

      QuadrupedTeleopDesiredPose otherMyClass = (QuadrupedTeleopDesiredPose) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.pose_.equals(otherMyClass.pose_)) return false;
      if(this.pose_shift_time_ != otherMyClass.pose_shift_time_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedTeleopDesiredPose {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("pose=");
      builder.append(this.pose_);      builder.append(", ");
      builder.append("pose_shift_time=");
      builder.append(this.pose_shift_time_);
      builder.append("}");
      return builder.toString();
   }
}

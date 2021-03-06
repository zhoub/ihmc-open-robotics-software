package us.ihmc.communication.packetCommunicator;

import us.ihmc.communication.blackoutGenerators.CommunicationBlackoutSimulator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;

public class BlackoutPacketConsumer<P extends Packet<P>> implements ObjectConsumer<P>
{
   private final PacketConsumer<P> packetConsumer;
   private CommunicationBlackoutSimulator blackoutSimulator;
   
   public BlackoutPacketConsumer(PacketConsumer<P> packetConsumer, CommunicationBlackoutSimulator blackoutSimulator)
   {
      this.packetConsumer = packetConsumer;
      this.blackoutSimulator = blackoutSimulator;
   }
   
   @Override
   public void consumeObject(P packet)
   {
      if(blackoutSimulator != null && !blackoutSimulator.blackoutCommunication())
         packetConsumer.receivedPacket(packet);
   }
}

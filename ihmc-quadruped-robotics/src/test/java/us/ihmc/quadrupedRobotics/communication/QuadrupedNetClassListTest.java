package us.ihmc.quadrupedRobotics.communication;

import org.junit.jupiter.api.Test;
import org.reflections.Reflections;
import us.ihmc.communication.kryo.KryoNetClassListTestHelper;
import us.ihmc.communication.net.NetClassList;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.pubsub.TopicDataType;

import java.util.Set;

public class QuadrupedNetClassListTest
{
   public static void main(String[] args)
   {
      Reflections ref = new Reflections();
      Set<Class<? extends TopicDataType>> subTypesOf = ref.getSubTypesOf(TopicDataType.class);

      for (Class<? extends TopicDataType> subTypeOf : subTypesOf)
         System.out.println("                         registerPacketField(" + subTypeOf.getSimpleName() + ".class);");
   }

   @Test // timeout = 30000
   public void testAllClassesRegisteredArePackets()
   {
      NetClassList netClassList = new QuadrupedNetClassList();
      KryoNetClassListTestHelper.testAllClassesRegisteredArePackets(netClassList);
   }

   @Test // timeout = 30000
   public void testAllPacketFieldsAreRegistered()
         throws InstantiationException, IllegalAccessException, NoSuchFieldException, SecurityException, IllegalArgumentException
   {
      NetClassList netClassList = new QuadrupedNetClassList();
      KryoNetClassListTestHelper.testAllPacketFieldsAreRegistered(netClassList);
   }
}

package us.ihmc.quadrupedRobotics.communication;

import java.util.Set;

import org.junit.jupiter.api.Test;
import org.reflections.Reflections;

import us.ihmc.communication.kryo.KryoNetClassListTestHelper;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.pubsub.TopicDataType;

public class QuadrupedNetClassListTest
{
   public static void main(String[] args)
   {
      Reflections ref = new Reflections();
      Set<Class<? extends TopicDataType>> subTypesOf = ref.getSubTypesOf(TopicDataType.class);

      for (Class<? extends TopicDataType> subTypeOf : subTypesOf)
         System.out.println("                         registerPacketField(" + subTypeOf.getSimpleName() + ".class);");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testAllClassesRegisteredArePackets()
   {
      NetClassList netClassList = new QuadrupedNetClassList();
      KryoNetClassListTestHelper.testAllClassesRegisteredArePackets(netClassList);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.5)
   @Test
   public void testAllPacketFieldsAreRegistered()
         throws InstantiationException, IllegalAccessException, NoSuchFieldException, SecurityException, IllegalArgumentException
   {
      NetClassList netClassList = new QuadrupedNetClassList();
      KryoNetClassListTestHelper.testAllPacketFieldsAreRegistered(netClassList);
   }
}

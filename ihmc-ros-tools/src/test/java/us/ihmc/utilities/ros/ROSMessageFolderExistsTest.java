package us.ihmc.utilities.ros;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ROSMessageFolderExistsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testROSMessageFolderExists()
   {
//      assertTrue(Files.exists(Paths.get(ROSMessageFileCreator.messageRootFolder)));
   }
}

package us.ihmc.utilities.ros;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class ROSMessageFolderExistsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testROSMessageFolderExists()
   {
//      assertTrue(Files.exists(Paths.get(ROSMessageFileCreator.messageRootFolder)));
   }
}

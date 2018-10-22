package us.ihmc.avatar;

import static org.junit.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.networkProcessor.modules.uiConnector.PacketsForwardedToTheUi;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class DRCConfigParametersTest
{
	@Test
   public void test()
   {
      assertFalse("Do not check in PacketsForwardedToTheUi.SEND_HIGH_SPEED_CONFIGURATION_DATA < 100!!", PacketsForwardedToTheUi.UI_JOINT_CONFIGURATION_UPDATE_MILLIS < 100);
   }
}

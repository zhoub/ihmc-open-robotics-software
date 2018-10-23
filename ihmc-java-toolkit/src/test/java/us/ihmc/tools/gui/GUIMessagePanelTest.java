package us.ihmc.tools.gui;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;

@Tag("ui")
public class GUIMessagePanelTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void testGetText()
   {
      GUIMessagePanel guiMessagePanel = new GUIMessagePanel("Test");
      
      guiMessagePanel.appendMessage("message0");
      guiMessagePanel.appendMessage("message1");
      
      String text = guiMessagePanel.getText();
            
      assertEquals("message1\nmessage0\n", text);
   }
}

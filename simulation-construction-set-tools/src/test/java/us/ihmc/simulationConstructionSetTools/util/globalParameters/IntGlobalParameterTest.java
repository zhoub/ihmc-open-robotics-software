package us.ihmc.simulationConstructionSetTools.util.globalParameters;


import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import static org.junit.Assert.assertEquals;

public class IntGlobalParameterTest
{
   private static final boolean VERBOSE = false;
   private final int DEFAULT_VALUE = 11;

   @BeforeEach
   public void setUp() throws Exception
   {
      GlobalParameter.clearGlobalRegistry();
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      GlobalParameter.clearGlobalRegistry();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test
   public void testGetValue()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

      IntGlobalParameter intGlobalParameter = new IntGlobalParameter("testParameter", "test description", DEFAULT_VALUE,
                                                 systemOutGlobalParameterChangedListener);
      assertEquals(DEFAULT_VALUE, intGlobalParameter.getValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test
   public void testSetValue()
   {
      SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = null;
      if (VERBOSE) systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();


      IntGlobalParameter intGlobalParameter = new IntGlobalParameter("testParameter", "test description", DEFAULT_VALUE,
                                                 systemOutGlobalParameterChangedListener);

      int newValue = -1;
      intGlobalParameter.set(newValue);
      assertEquals(newValue, intGlobalParameter.getValue());

      newValue = 1100;
      intGlobalParameter.set(newValue, "setting");
      assertEquals(newValue, intGlobalParameter.getValue());

      newValue = 1100;
      intGlobalParameter.setOnlyIfChange(newValue, "setting");
      assertEquals(newValue, intGlobalParameter.getValue());

      newValue = -906;
      intGlobalParameter.setOnlyIfChange(newValue, "setting");
      assertEquals(newValue, intGlobalParameter.getValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test
   public void testThatCantHaveParentsUnlessOverwriteUpdateMethodOne()
   {
      IntGlobalParameter parent = new IntGlobalParameter("parent", "parent", DEFAULT_VALUE, null);
      @SuppressWarnings("unused")
      IntGlobalParameter invalidChild = new IntGlobalParameter("invalidChild", "test description", new GlobalParameter[] {parent}, null);

      parent.set(1);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test
   public void testCantSetChild()
   {
      IntGlobalParameter parent = new IntGlobalParameter("parent", "", 0, null);
      IntGlobalParameter child = new IntGlobalParameter("child", "", new GlobalParameter[] {parent}, null);

      child.set(2, "Shouldn't be able to change this!");
   }
}

package us.ihmc.tools.compression;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class LZ4CompressionImplementationTest
{
   @Test // timeout = 30000
   public void testLength()
   {
      LZ4CompressionImplementation impl = new LZ4CompressionImplementation();

      Random random = new Random(12597651l);
      for (int i = 0; i < 1000; i++)
      {
         int test = random.nextInt(65000);
         int max = impl.maxCompressedLength(test);
         int min = impl.minimumDecompressedLength(max);

         assertTrue("Got: " + min + ", expected " + test + " or " + test + "-1", test == min || (test - 1) == min);

      }

   }
}

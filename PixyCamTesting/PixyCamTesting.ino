#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();

  // This header labels the data streams in the Serial Plotter window.
  Serial.println("X,Y");
}

void loop()
{ 
  // Grab the detected blocks from Pixy
  pixy.ccc.getBlocks();
  
  // If any blocks are detected, send the X and Y of the largest one (block 0)
  if (pixy.ccc.numBlocks > 0)
  {
    // Print the X coordinate, followed by a comma
    Serial.print(pixy.ccc.blocks[0].m_x);
    Serial.print(",");
    
    // Print the Y coordinate, followed by a newline to complete the data point
    Serial.println(pixy.ccc.blocks[0].m_y);
  }
}
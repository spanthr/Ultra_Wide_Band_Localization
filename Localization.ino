// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
// Author: Shaurya Panthri || Chinmay Rathod 

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint8_t num_anchors = 15;                                    // the number of anchors
//uint16_t anchors[15] = {0x6a26, 0x6a2d, 0x6a30, 0x6a31, 0x6a38, 0x6a39, 0x6a3b, 0x6a40, 0x6a4a, 0x6a41, 0x6a33, 0x6a20, 0x6043, 0x6a6d, 0x6a44};     // the network id of the anchors: change these to the network ids of your anchors.
//uint16_t anchors[15] = {0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx, 0x6axx;     // the network id of the anchors: change these to the network ids of your anchors.
int32_t heights[15] = {3750,3750,5000,3750,3750,3750,3750,3750,3750,3750,3750,3750,3750,3750,3750};              // anchor z-coordinates in mm
boolean bProcessing = false;                                // set this to true to output data for the processing sketch         

// only required for manual anchor calibration. Please change this to the coordinates measured for the anchors
int32_t anchors_x[15] = {66810,66703,45096,66925,67236,66719,39674,66851,38630,-148215,69714,55518,55043,63608,65649};              // anchor x-coorindates in mm
int32_t anchors_y[15] = {-23160,25000,-97204,-58698,977,-114712,-48892,-82832,35527,-148215,-196911,-196850,-172626,100955,50845};                  // anchor y-coordinates in mm

////////////////////////////////////////////////


void setup(){
  Serial.begin(115200);
  
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }
  
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println(F("NOTES:"));
  Serial.println(F("- No parameters required."));
  Serial.println();
  Serial.println(F("- System will auto start calibration"));
  Serial.println();
  Serial.println(F("- System will auto start positioning"));
  Serial.println(F("----------POZYX POSITIONING V1.0----------"));
  Serial.println();
  Serial.println(F("Performing auto anchor calibration:"));

  // clear all previous devices in the device list
  Pozyx.clearDevices();
     
  //int status = Pozyx.doAnchorCalibration(POZYX_2_5D, 10, num_anchors, anchors, heights);
  //if (status != POZYX_SUCCESS){
  // Serial.println(status);
  //Serial.println(F("ERROR: calibration"));
  // Serial.println(F("Reset required"));
  // delay(100);
  // abort();
  //}
  
  // if the automatic anchor calibration is unsuccessful, try manually setting the anchor coordinates.
  // fot this, you must update the arrays anchors_x, anchors_y and heights above
  // comment out the doAnchorCalibration block and the if-statement above if you are using manual mode
  SetAnchorsManual();

  printCalibrationResult();
  delay(3000);

  Serial.println(F("Starting positioning: "));

}

void loop(){
  
  coordinates_t position;  
  int status = Pozyx.doPositioning(&position, POZYX_3D, 2000);
  euler_angles_t hdg;
  Pozyx.getEulerAngles_deg(&hdg,NULL);
  if (status == POZYX_SUCCESS)
  {
    // print out the result
    if(!bProcessing){
      printCoordinates(position, hdg);
    }else{    
      printCoordinates(position, hdg);
    }
  }
}

// function to print the coordinates to the serial monitor
void printCoordinates(coordinates_t coor, euler_angles_t hdg){
  
  Serial.print("x_mm: ");
  Serial.print(coor.x);
  Serial.print("||\t");
  Serial.print("y_mm: ");
  Serial.print(coor.y);
  Serial.print("||\t");
  Serial.print("z_mm: ");
  Serial.print(coor.z);
  Serial.println(); 
  Serial.print("\t || h_deg: ");
  Serial.println((int) hdg.heading);
  Serial.println();
}

// function to print out positoining data + ranges for the processing sketch
void printCoordinatesProcessing(coordinates_t coor){
  
  // get the network id and print it
  uint16_t network_id;
  Pozyx.getNetworkId(&network_id);
  
  Serial.print("POS,0x");
  Serial.print(network_id,HEX);
  Serial.print("||");
  Serial.print(coor.x);
  Serial.print("||");
  Serial.print(coor.y);
  Serial.print("||");
  Serial.print(coor.z);
  Serial.print("||");
  
  // get information about the positioning error and print it
  pos_error_t pos_error;
  Pozyx.getPositionError(&pos_error);
    
  Serial.print(pos_error.x);
  Serial.print("||");
  Serial.print(pos_error.y);
  Serial.print("||");
  Serial.print(pos_error.z);
  Serial.print("||");
  Serial.print(pos_error.xy);
  Serial.print("||");
  Serial.print(pos_error.xz);
  Serial.print("||");
  Serial.print(pos_error.yz); 
  
  // read out the ranges to each anchor and print it 
  for (int i=0; i < num_anchors; i++){
    device_range_t range;
    Pozyx.getDeviceRangeInfo(anchors[i], &range);
    Serial.print("||");
    Serial.print(range.distance);  
    Serial.print("||");
    Serial.print(range.RSS); 
  }
  Serial.println();
}

// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size);
  Serial.print("list size: ");
  Serial.println(status*list_size);
  
  if(list_size == 0){
    Serial.println("Calibration failed.");
    Serial.println(Pozyx.getSystemError());
    return;
  }
  
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids,list_size);
  
  Serial.println(F("Calibration result:"));
  Serial.print(F("Anchors found: "));
  Serial.println(list_size);
  
  coordinates_t anchor_coor;
  for(int i=0; i<list_size; i++)
  {
    
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print("||");    
    status = Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor);
    Serial.print(anchor_coor.x);
    Serial.print("||");
    Serial.print(anchor_coor.y);
    Serial.print("||");
    Serial.println(anchor_coor.z);
    Serial.print("||");
   
  }    
}

// function to manually set the anchor coordinates
void SetAnchorsManual(){
 
 int i=0;
 for(i=0; i<num_anchors; i++){
   device_coordinates_t anchor;
   anchor.network_id = anchors[i];
   anchor.flag = 0x1; 
   anchor.pos.x = anchors_x[i];
   anchor.pos.y = anchors_y[i];
   anchor.pos.z = heights[i];
   Pozyx.addDevice(anchor);
 }
 
}

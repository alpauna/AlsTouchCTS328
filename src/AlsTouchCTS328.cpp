#include "Touch_CST328.h"

struct CST328_Touch touch_data = {0};
struct CST328_Touch Touch = {0};
uint8_t _resetPin = 12;
uint8_t _intPin = 13;
void Als_Tch_CST328(uint8_t resetPin, uint8_t intPin, uint8_t sda, uint8_t scl, uint32_t frequency) {
  _resetPin = resetPin;
  _intPin = intPin;
  Wire.begin(sda, scl, frequency);
  pinMode(_intPin, INPUT_PULLUP);
  pinMode(_resetPin, OUTPUT);

  CST328_Touch_Reset();
  uint16_t Verification = CST328_Read_cfg();
  if(!((Verification==0xCACA)?true:false))
  Serial.print("Touch initialization failed!\r\n");

  attachInterrupt(digitalPinToInterrupt(_intPin), Touch_CST328_ISR, interrupt); //Configure Touch Interupt Function

  return ((Verification==0xCACA)?true:false);
}
/* Reset controller */
uint8_t CST328_Touch_Reset(void)
{
    digitalWrite(_resetPin, HIGH );     // Reset
    delay(50);
    digitalWrite(_resetPin, LOW);
    delay(5);
    digitalWrite(_resetPin, HIGH );
    delay(50);
    return true;
}
uint16_t CST328_Read_cfg(void) {

  uint8_t buf[24];
  I2C_Write(CST328_ADDR, HYN_REG_MUT_DEBUG_INFO_MODE, buf, 0);
  I2C_Read(CST328_ADDR, HYN_REG_MUT_DEBUG_INFO_BOOT_TIME,buf, 4);
  Serial.println("TouchPad_ID:0x" + String(buf[0],HEX) + ",0x" + String(buf[1],HEX) + ",0x" + String(buf[2],HEX) + ",0x" + String(buf[3],HEX));
  I2C_Read(CST328_ADDR, HYN_REG_MUT_DEBUG_INFO_RES_X, buf, 4);
  Serial.print("TouchPad_X_MAX:" + String(buf[1]*256+buf[0]) + "    TouchPad_Y_MAX:" + String(buf[3]*256+buf[2]) + "\r\n");

  I2C_Read(CST328_ADDR, HYN_REG_MUT_DEBUG_INFO_TP_NTX, buf, 24);
  Serial.println("D1F4:0x" + String(buf[0], HEX) + ",0x" + String(buf[1], HEX) + ",0x" + String(buf[2], HEX) + ",0x" + String(buf[3], HEX));
  Serial.println("D1F8:0x" + String(buf[4], HEX) + ",0x" + String(buf[5], HEX) + ",0x" + String(buf[6], HEX) + ",0x" + String(buf[7], HEX));
  Serial.println("D1FC:0x" + String(buf[8], HEX) + ",0x" + String(buf[9], HEX) + ",0x" + String(buf[10], HEX) + ",0x" + String(buf[11], HEX));
  Serial.println("D200:0x" + String(buf[12], HEX) + ",0x" + String(buf[13], HEX) + ",0x" + String(buf[14], HEX) + ",0x" + String(buf[15], HEX));
  Serial.println("D204:0x" + String(buf[16], HEX) + ",0x" + String(buf[17], HEX) + ",0x" + String(buf[18], HEX) + ",0x" + String(buf[19], HEX));
  Serial.println("D208:0x" + String(buf[20], HEX) + ",0x" + String(buf[21], HEX) + ",0x" + String(buf[22], HEX) + ",0x" + String(buf[23], HEX));
  Serial.println("CACA Read:0x" + String((((uint16_t)buf[11] << 8) | buf[10]), HEX));

  I2C_Write(CST328_ADDR, HYN_REG_MUT_NORMAL_MODE, buf, 0);
  return (((uint16_t)buf[11] << 8) | buf[10]);
}

// reads sensor and touches
// updates Touch Points, but if not touched, resets all Touch Point Information
uint8_t Touch_Read_Data(void) {
  uint8_t buf[41];
  uint8_t touch_cnt = 0;
  uint8_t clear = 0;
  uint8_t Over = 0xAB;
  size_t i = 0,num=0;
  I2C_Read(CST328_ADDR, ESP_LCD_TOUCH_CST328_READ_Number_REG, buf, 1);
  if ((buf[0] & 0x0F) == 0x00) {                                              // Determine the number of fingers
    I2C_Write(CST328_ADDR, ESP_LCD_TOUCH_CST328_READ_Number_REG, &clear, 1);  // No touch data
  } else {
    /* Count of touched points */
    touch_cnt = buf[0] & 0x0F;
    if (touch_cnt > CST328_LCD_TOUCH_MAX_POINTS || touch_cnt == 0) {
      I2C_Write(CST328_ADDR, ESP_LCD_TOUCH_CST328_READ_Number_REG, &clear, 1);
      return true;
    }
    /* Read all points */
    I2C_Read(CST328_ADDR, ESP_LCD_TOUCH_CST328_READ_XY_REG, &buf[1], 27);
    /* Clear all */
    I2C_Write(CST328_ADDR, ESP_LCD_TOUCH_CST328_READ_Number_REG, &clear, 1);
    // Serial.print(" points=%d \r\n",touch_cnt);
    noInterrupts(); // Enter the critical section
      // Execute the code that needs to be protected in the critical section
    /* Number of touched points */
    if(touch_cnt > CST328_LCD_TOUCH_MAX_POINTS)
        touch_cnt = CST328_LCD_TOUCH_MAX_POINTS;
    touch_data.points = (uint8_t)touch_cnt;
    /* Fill all coordinates */
    for (i = 0; i < touch_cnt; i++) {
      if(i>0) num = 2;
      touch_data.coords[i].x = (uint16_t)(((uint16_t)buf[(i * 5) + 2 + num] << 4) + ((buf[(i * 5) + 4 + num] & 0xF0)>> 4));               
      touch_data.coords[i].y = (uint16_t)(((uint16_t)buf[(i * 5) + 3 + num] << 4) + ( buf[(i * 5) + 4 + num] & 0x0F));
      touch_data.coords[i].strength = ((uint16_t)buf[(i * 5) + 5 + num]);
    }
    interrupts(); // Exit critical section
    // Serial.print(" points=%d \r\n",touch_data.points);
  }
  return true;
}
void Touch_Loop(void){
  if(Touch_interrupts){
    Touch_interrupts = false;
    example_touchpad_read();
  }
}
/*!
    @brief  Get the finger coordinate position
	@param	uint16_t *x
			Store the X coordinates of each finger
	@param	uint16_t *y
			Store the Y coordinates of each finger
	@param	uint16_t *strength
			Store the pressure of each finger
	@param	uint8_t *point_num
			Pointer to store the number of fingers
	@param	uint8_t max_point_num
			Maximum number of fingers
*/
uint8_t Touch_Get_XY(uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num) {
  
  noInterrupts(); // Enter the critical section
      // Execute the code that needs to be protected in the critical section
  /* Count of points */
  if(touch_data.points > max_point_num)
    touch_data.points = max_point_num;
  for (size_t i = 0; i < touch_data.points; i++) {
      x[i] = touch_data.coords[i].x;
      y[i] = touch_data.coords[i].y;
      if (strength) {
          strength[i] = touch_data.coords[i].strength;
      }
  }
  *point_num = touch_data.points;
  /* Invalidate */
  touch_data.points = 0;
  interrupts(); // Exit critical section
  return (*point_num > 0);
}
void example_touchpad_read(void){
  uint16_t touchpad_x[5] = {0};
  uint16_t touchpad_y[5] = {0};
  uint16_t strength[5]   = {0};
  uint8_t touchpad_cnt = 0;
  Touch_Read_Data();
  uint8_t touchpad_pressed = Touch_Get_XY(touchpad_x, touchpad_y, strength, &touchpad_cnt, CST328_LCD_TOUCH_MAX_POINTS);
  if (touchpad_pressed && touchpad_cnt > 0) {
      Touch.points = touchpad_cnt;
      for (size_t i = 0; i <touchpad_cnt; i++) {
        Touch.coords[i].x = touchpad_x[i];
        Touch.coords[i].y = touchpad_y[i];
      }
      Touch.state = LV_INDEV_STATE_PR;
      Serial.println("Touch : X=" + String(touchpad_x[0]) + " Y=" + String(touchpad_y[0]) +" num=" + String(touchpad_cnt));
  } else {
      Touch.state  = LV_INDEV_STATE_REL;
  }
}
/*!
    @brief  handle interrupts
*/
uint8_t Touch_interrupts = 0;
void Touch_CST328_ISR(void) {
  Touch_interrupts = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// I2C Read and Write
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool I2C_Read(uint8_t Driver_addr, uint16_t Reg_addr, uint8_t *Reg_data, uint32_t Length)
{
  Wire.beginTransmission(Driver_addr);
  Wire.write((uint8_t)(Reg_addr >> 8)); // High 8 bits of address
  Wire.write((uint8_t)Reg_addr);         // Lower 8 bits of address
  if ( Wire.endTransmission(true)){
    Serial.print("The I2C transmission fails. - I2C Read\r\n");
    return -1;
  }
  Wire.requestFrom(Driver_addr, Length);
  for (int i = 0; i < Length; i++) {
    *Reg_data++ = Wire.read();
  }
  return 0;
}
bool I2C_Write(uint8_t Driver_addr, uint16_t Reg_addr, const uint8_t *Reg_data, uint32_t Length)
{
  Wire.beginTransmission(Driver_addr);
  Wire.write((uint8_t)(Reg_addr >> 8)); // High 8 bits of address
  Wire.write((uint8_t)Reg_addr);         // Lower 8 bits of address
  for (int i = 0; i < Length; i++) {
    Wire.write(*Reg_data++);
  }
  if ( Wire.endTransmission(true))
  {
    Serial.print("The I2C transmission fails. - I2C Write\r\n");
    return -1;
  }
  return 0;
}
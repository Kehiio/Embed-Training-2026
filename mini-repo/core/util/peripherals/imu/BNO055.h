// THIS IS THE HEADER FOR YOUR WEEK 2 ASSIGNMENT. DO NOT CHANGE ANYTHING ALREADY WRITTEN, BUT YOU ARE FREE TO ADD YOUR OWN FUNCTIONS
// Make sure to implement all functions listed 
#ifndef BNO055_H
#define BNO055_H

#include "mbed.h"
#include "IMU.h"
#include <mbed-os/thisThread.cpp>
#define PI 3.14159265
#define LA_CONVERSION_FACTOR 0.01
#define GYR_CONVERSION_DEG 0.0625

// Registers
#define LIA_Data_X_LSB 0x28 
#define LIA_Data_Z_MSB 0x2D

#define GYR_DATA_X_LSB 0x14
#define GYR_DATA_Z_MSB 0x19

#define OPERATING_MODE_REG 0x3D
#define M_IMU 0x08
#define M_COMPASS 0x09
#define M_M4G 0x0A
#define M_NDOF_FMC_OFF 0x0B
#define M_NDOF 0x0C


typedef struct{
    double yaw;
    double roll;
    double pitch;
} BNO055_ANGULAR_POSITION_typedef;

typedef struct {
    double x;
    double y;
    double z;
} BNO055_VECTOR_TypeDef;

typedef struct {
    double x;
    double y;
    double z;
    double w;
} BNO055_QUATERNION_TypeDef;

class BNO055: public IMU
{
public:

    public:
    BNO055(I2C &i2c, uint8_t addr) noexcept; // Constructor taking in i2c object and address

    BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept; // Overloaded to take a specific reset pin 
                                                              // Implement both of these in the cpp

    void init() noexcept; // Initialize the BNO055    

    void reset() noexcept; // Quickly reset the BNO055

    /** Get Accel data
     * @param double type of 3D data address
     */
    void get_accel(BNO055_VECTOR_TypeDef *la);

    /** Get Gyro data
     * @param double type of 3D data address
     */
    void get_gyro(BNO055_VECTOR_TypeDef *gr);

    /** Change fusion mode
      * @param fusion mode
      * @return none
      */
    void change_fusion_mode(uint8_t mode);

    IMU::EulerAngles read() override;

    IMU::EulerAngles getImuAngles() override;

    /** Get Quaternion XYZ&W
     * @param int16_t type of 4D data address
     */
    void get_quaternion(BNO055_QUATERNION_TypeDef *qua);

    /** Get Angular position from quaternion
     *  @param double type of 3D data address
     */
    void get_angular_position_quat(IMU::EulerAngles *an_pos);

protected:

    I2C *_i2c_p;            // Pointer to I2C communication obj
    I2C &_i2c;              // Reference to I2C communication obj
    DigitalOut _res;        // working buffer 
    uint8_t  chip_addr;     // IMU's chip address
    int cantReadDataCount;       
    
private:

    char     dt[10];        // working buffer
    IMU::EulerAngles imuAngles;

    

};

// Quaternion data registers
#define BNO055_QUATERNION_W_LSB 0x20

#endif
#include "BNO055.h"
#include "thisThread.cpp"
#include <cstddef>

/* ---------- CONSTRUCTORS ---------- */

/* I2C &i2c         - IN    I2C object reference for communication
   uint8_t addr     - IN    This part's i2c communication address (8 bit)

*/
BNO055::BNO055(I2C &i2c, uint8_t addr) noexcept : _i2c(i2c), _res(NC), dt{}, chip_addr(addr) {
    cantReadDataCount = 0;
    *_i2c_p = i2c;
    _i2c.frequency(100000);
    init();

}

/* I2C &i2c         - IN    I2C object reference for communication
   uint8_t addr     - IN    This part's i2c communication address (8 bit)
   PinName p_reset  - IN    Identifier for reset pin 
*/
BNO055::BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept : 
_i2c(i2c), _res(p_reset), chip_addr(addr), dt{}  {
    cantReadDataCount = 0;
    *_i2c_p = i2c;
    _i2c.frequency(100000);
    init();

}

/* ---------- PUBLICS ---------- */

/* init(): Initialize BNO055 to IMU fusion mode */
void BNO055::init() noexcept {
    // Wait for boot
    ThisThread::sleep_for(800);

    // check address for communication is correct


    // quick reset on chip incase power wasn't lost
    reset();

    change_fusion_mode(M_IMU);

    return;
}

/* reset(): Quickly reset BNO055 */
void BNO055::reset() noexcept {
    
    _res = 0;   // set low
    ThisThread::sleep_for(5);

    _res = 1;   // set high
    ThisThread::sleep_for(800);
    cantReadDataCount = 0;

    // check chip ID

    return;
}

/* BNO055_VECTOR_TypeDef *la - OUT  return custom vector containing linear acceleration data */
void BNO055::get_accel(BNO055_VECTOR_TypeDef *la){
    
    // Error escape
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }


    dt[0] = LIA_Data_X_LSB;
    int16_t la_x, la_y, la_z;
    
    // Ping desired register in write mode (default, LSB==0)
    int writeResult = _i2c_p->write(chip_addr, dt, 1, true);

    if (!writeResult)
    {
        // read desired register in read mode (LSB==1) to data buffer (dt)
        // registers 0x28 to 0x2D
        if(_i2c_p->read((chip_addr | 0b1), dt, 6, false) != 0){
            cantReadDataCount++;
            return;
        }

        // Combine 2 the two bytes of data per direction
        la_x = (int16_t)(dt[1] << 8) | dt[0];
        la_y = (int16_t)(dt[3] << 8) | dt[2];
        la_z = (int16_t)(dt[5] << 8) | dt[4];

        // unit conversion to m/s^2
        la_x *= LA_CONVERSION_FACTOR;
        la_y *= LA_CONVERSION_FACTOR;
        la_z *= LA_CONVERSION_FACTOR;

        // Put m/s^2 linear acceleration data into return variable
        la->x = la_x;
        la->y = la_y;
        la->z = la_z;
    } else {
        cantReadDataCount++;
    }

    return;
}

/* BNO055_VECTOR_TypeDef *gr - OUT  return custom vector containing gyro data */
void BNO055::get_gyro(BNO055_VECTOR_TypeDef *gr){
    
    // Error escape
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }

    // Starting register for GYR data
    dt[0] = GYR_DATA_X_LSB;
    
    // Ping desired register in write mode (default, LSB==0)
    int writeResult =_i2c_p->write(chip_addr, dt, 1, true);

    if (!writeResult){
        // read desired register in read mode (LSB==1) to data buffer (dt)
        // registers 0x28 to 0x2D
        if(_i2c_p->read((chip_addr | 0b1), dt, 6, false) != 0){
            cantReadDataCount++;
            return;
        }

        // Combine 2 the two bytes of data per direction
        double gyr_x = (int16_t)(dt[1] << 8) | dt[0];
        double gyr_y = (int16_t)(dt[3] << 8) | dt[2];
        double gyr_z = (int16_t)(dt[5] << 8) | dt[4];

        // unit conversion to deg/sec
        gyr_x *= GYR_CONVERSION_DEG;
        gyr_y *= GYR_CONVERSION_DEG;
        gyr_z *= GYR_CONVERSION_DEG;

        // Put deg/s gyro data into return variable
        gr->x = gyr_x;
        gr->y = gyr_x;
        gr->z = gyr_x;

        return;
    } else {
        cantReadDataCount++;
    }

}

/** Change fusion mode
    * @param fusion mode
    * @return none
    */
void BNO055::change_fusion_mode(uint8_t mode){
    // check if input mode is a valid fusion mode
    if (mode == M_IMU           || 
        mode == M_COMPASS       || 
        mode == M_M4G           || 
        mode == M_NDOF_FMC_OFF  || 
        mode == M_NDOF){

        char write_reg[2] = {OPERATING_MODE_REG, 0x0};
        
        // Enter config mode by writting 0 to op register
        _i2c_p->write(chip_addr, write_reg, 1, false);
        ThisThread::sleep_for(20); 

        // write change mode to register address
        write_reg[1] = (char)mode;

        _i2c_p->write(chip_addr, write_reg, 1, false);
        ThisThread::sleep_for(20);
    }
    else{
        printf("[WAR] Fusion mode change failed, mode %d unknown\n", mode);
    }

    return;
}


IMU::EulerAngles BNO055::read(){
    

}

IMU::EulerAngles BNO055::getImuAngles(){
    imuAngles = read();

    return imuAngles;
}

/*
We added these two functions since they're definitely beyond what we expect from you as recruits
I'd be impressed if you understood the math behind them.
Basically just know that quats are 4 axis (1 real, 3 imaginary) numbers that allow us 
to nicely talk about rotation, and the functions spit out pitch roll and yaw. 
*/
void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
{
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t w,x,y,z;

    dt[0] = BNO055_QUATERNION_W_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 8, false);
        w = dt[1] << 8 | dt[0];
        x = dt[3] << 8 | dt[2];
        y = dt[5] << 8 | dt[4];
        z = dt[7] << 8 | dt[6];

        result->w = (double)w / 16384.0f;
        result->x = (double)x / 16384.0f;
        result->y = (double)y / 16384.0f;
        result->z = (double)z / 16384.0f;
    } else {
        cantReadDataCount++;
    }
}

void BNO055::get_angular_position_quat(IMU::EulerAngles *result){

    BNO055_QUATERNION_TypeDef q;
    get_quaternion(&q);

    float roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
    float pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
    float yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;

    memcpy(&result->roll, &roll, sizeof(float));
    memcpy(&result->pitch, &pitch, sizeof(float));
    memcpy(&result->yaw, &yaw, sizeof(float));
}



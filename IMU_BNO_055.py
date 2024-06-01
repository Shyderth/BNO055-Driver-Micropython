from machine import Pin, I2C
from micropython import const
import time

#Direccion del BNO_055
MPU_ADDRESS = const(0x28)

PWR_MODE = const(0x3E)     #Determina el consumo de energia y el estado de hibernacion
OPR_MODE = const(0x3D)     #Determina el modo de operacion del BNO
UNIT_SEL = const(0x3B)     #Seleccion de unidades (ACC, MAG, GYR, TEMP, EUL_ORIENTATION(Android/windows)) lsb->msb

ST_RESULT = const(0x36)    #Muestra el resultado del self-test (1=OK, 0=BAD) ejecutado el iniciarse la IMU
CALIB_STAT = const(0x35)   #Muestra el estado de la calibracion de los sensores

#Registro para el sensor de Temperatura
TEMP = const(0x34)

#Registros del acelerometro
ACC_DATA_X_LSB = const(0x8)
ACC_DATA_X_MSB = const(0x9)
ACC_DATA_Y_LSB = const(0xA)
ACC_DATA_Y_MSB = const(0xB)
ACC_DATA_Z_LSB = const(0xC)
ACC_DATA_Z_MSB = const(0xD)

#Registros del magnetometro
MAG_DATA_X_LSB = const(0xE)
MAG_DATA_X_MSB = const(0xF)
MAG_DATA_Y_LSB = const(0x10)
MAG_DATA_Y_MSB = const(0x11)
MAG_DATA_Z_LSB = const(0x12)
MAG_DATA_Z_MSB = const(0x13)

#Registros del Giroscopio
GYR_DATA_X_LSB = const(0x14)
GYR_DATA_X_MSB = const(0x15)
GYR_DATA_Y_LSB = const(0x16)
GYR_DATA_Y_MSB = const(0x17)
GYR_DATA_Z_LSB = const(0x18)
GYR_DATA_Z_MSB = const(0x19)

#Registros para Angulos de Euler
EUL_HEADING_LSB = const(0x1A)
EUL_HEADING_MSB = const(0x1B)
EUL_ROLL_LSB = const(0x1C)
EUL_ROLL_MSB = const(0x1D)
EUL_PITCH_LSB = const(0x1E)
EUL_PITCH_MSB = const(0x1F)

#Registros para Cuaterniones
QUA_DATA_W_LSB = const(0x20)
QUA_DATA_W_MSB = const(0x21)
QUA_DATA_X_LSB = const(0x22)
QUA_DATA_X_MSB = const(0x23)
QUA_DATA_Y_LSB = const(0x24)
QUA_DATA_Y_MSB = const(0x25)
QUA_DATA_Z_LSB = const(0x26)
QUA_DATA_Z_MSB = const(0x27)


class BNO_055_I2C:
    
    def __init__(self, i2c, mode: str):
        self.i2c = i2c
        self.mode = mode
        #Se establece en modo configuracion para poder hacer el cambio de modos
        self.i2c.writeto_mem(MPU_ADDRESS, OPR_MODE, bytes([0]))
        time.sleep(0.5)
        if (self.mode.upper() is "CONFIG"):
            print("Modo de Configracion establecido")
            self.i2c.writeto_mem(MPU_ADDRESS, OPR_MODE, bytes([0]))
        elif (self.mode.upper() is "AMG"):
            print("Modo AMG configurado")
            self.i2c.writeto_mem(MPU_ADDRESS, OPR_MODE, bytes([7]))
        elif (self.mode.upper() is "IMU"):
            print("MODO IMU configurado")
            self.i2c.writeto_mem(MPU_ADDRESS, OPR_MODE, bytes([8]))
        elif (self.mode.upper() is "NDOF"):
            print("MODO NDOF configurado")
            self.i2c.writeto_mem(MPU_ADDRESS, OPR_MODE, bytes([12]))
        else:
            print("Modo no reconocido, modo por defecto AMG establecido")
            self.i2c.writeto_mem(MPU_ADDRESS, OPR_MODE, bytes([7]))
        time.sleep(0.5)
        
        st_Res = i2c.readfrom_mem(MPU_ADDRESS, ST_RESULT,1)[0]
        if st_Res is 15:
            print("Todos los sistemas funcionando")
        elif st_Res is 14:
            print("WARNING: Falla en el acelerometro!")
        elif st_Res is 13:
            print("WARNING: Falla en el magnetometro!")
        elif st_Res is 11:
            print("WARNING: Falla en el gyroscopio!")
        else:
            print("WARNING: Falla en la unidad de procesamiento!")
        #print(i2c.readfrom_mem(MPU_ADDRESS, OPR_MODE, 1)[0])
        
        
    def int_from_bytes(self,msb, lsb):
        if not msb & 0x80:
            return msb << 8 | lsb
        
        return - (((msb ^ 255) << 8) | (lsb ^ 255) + 1)
    
    
    def get_accel(self):
        #TODO
        x_l = self.i2c.readfrom_mem(MPU_ADDRESS, ACC_DATA_X_LSB, 1);
        x_h = self.i2c.readfrom_mem(MPU_ADDRESS, ACC_DATA_X_MSB, 1);
        y_l = self.i2c.readfrom_mem(MPU_ADDRESS, ACC_DATA_Y_LSB, 1);
        y_h = self.i2c.readfrom_mem(MPU_ADDRESS, ACC_DATA_Y_MSB, 1);
        z_l = self.i2c.readfrom_mem(MPU_ADDRESS, ACC_DATA_Z_LSB, 1);
        z_h = self.i2c.readfrom_mem(MPU_ADDRESS, ACC_DATA_Z_MSB, 1);
        
        
        accel_x = self.int_from_bytes(x_h[0], x_l[0])/100
        accel_y = self.int_from_bytes(y_h[0], y_l[0])/100
        accel_z = self.int_from_bytes(z_h[0], z_l[0])/100
        
        return [accel_x, accel_y, accel_z]
    
    def get_gyro(self):
        #TODO
        x_l = self.i2c.readfrom_mem(MPU_ADDRESS, GYR_DATA_X_LSB, 1);
        x_h = self.i2c.readfrom_mem(MPU_ADDRESS, GYR_DATA_X_MSB, 1);
        y_l = self.i2c.readfrom_mem(MPU_ADDRESS, GYR_DATA_Y_LSB, 1);
        y_h = self.i2c.readfrom_mem(MPU_ADDRESS, GYR_DATA_Y_MSB, 1);
        z_l = self.i2c.readfrom_mem(MPU_ADDRESS, GYR_DATA_Z_LSB, 1);
        z_h = self.i2c.readfrom_mem(MPU_ADDRESS, GYR_DATA_Z_MSB, 1);
        
        gyro_x = self.int_from_bytes(x_h[0], x_l[0])/16
        gyro_y = self.int_from_bytes(y_h[0], y_l[0])/16
        gyro_z = self.int_from_bytes(z_h[0], z_l[0])/16
        
        return [gyro_x, gyro_y, gyro_z]
    
    def get_mag(self):
        #TODO
        x_l = self.i2c.readfrom_mem(MPU_ADDRESS, MAG_DATA_X_LSB, 1);
        x_h = self.i2c.readfrom_mem(MPU_ADDRESS, MAG_DATA_X_MSB, 1);
        y_l = self.i2c.readfrom_mem(MPU_ADDRESS, MAG_DATA_Y_LSB, 1);
        y_h = self.i2c.readfrom_mem(MPU_ADDRESS, MAG_DATA_Y_MSB, 1);
        z_l = self.i2c.readfrom_mem(MPU_ADDRESS, MAG_DATA_Z_LSB, 1);
        z_h = self.i2c.readfrom_mem(MPU_ADDRESS, MAG_DATA_Z_MSB, 1);
        #TODO: Revisar la ecuacion del magnetometro
        mag_x =self.int_from_bytes(x_h[0], x_l[0])/16
        mag_y = self.int_from_bytes(y_h[0], y_l[0])/16
        mag_z = self.int_from_bytes(z_h[0], z_l[0])/16
        
        return [mag_x, mag_y, mag_z]
    
    def get_Temp(self):
        return self.i2c.readfrom_mem(MPU_ADDRESS, TEMP, 1)[0]
    
    def get_Eul_Ang(self):
        #POR DEFECTO ESTA EN GRADOS (DEGREE)
        x_l = self.i2c.readfrom_mem(MPU_ADDRESS, EUL_ROLL_LSB, 1);
        x_h = self.i2c.readfrom_mem(MPU_ADDRESS, EUL_ROLL_MSB, 1);
        y_l = self.i2c.readfrom_mem(MPU_ADDRESS, EUL_PITCH_LSB, 1);
        y_h = self.i2c.readfrom_mem(MPU_ADDRESS, EUL_PITCH_MSB, 1);
        z_l = self.i2c.readfrom_mem(MPU_ADDRESS, EUL_HEADING_LSB, 1);
        z_h = self.i2c.readfrom_mem(MPU_ADDRESS, EUL_HEADING_MSB, 1);
        
        roll = self.int_from_bytes(x_h[0], x_l[0])/16 
        pitch = self.int_from_bytes(y_h[0], y_l[0])/16 
        yaw = self.int_from_bytes(z_h[0], z_l[0])/16 
        return [roll, pitch, yaw]
    
    def get_Quat(self):
        #TODO
        x_l = self.i2c.readfrom_mem(MPU_ADDRESS, QUA_DATA_X_LSB, 1);
        x_h = self.i2c.readfrom_mem(MPU_ADDRESS, QUA_DATA_X_MSB, 1);
        y_l = self.i2c.readfrom_mem(MPU_ADDRESS, QUA_DATA_Y_LSB, 1);
        y_h = self.i2c.readfrom_mem(MPU_ADDRESS, QUA_DATA_Y_MSB, 1);
        z_l = self.i2c.readfrom_mem(MPU_ADDRESS, QUA_DATA_Z_LSB, 1);
        z_h = self.i2c.readfrom_mem(MPU_ADDRESS, QUA_DATA_Z_MSB, 1);
        w_l = self.i2c.readfrom_mem(MPU_ADDRESS, QUA_DATA_W_LSB, 1);
        w_h = self.i2c.readfrom_mem(MPU_ADDRESS, QUA_DATA_W_MSB, 1);
        
        qua_x =self.int_from_bytes(x_h[0], x_l[0])/(2**14)
        qua_y = self.int_from_bytes(y_h[0], y_l[0])/(2**14)
        qua_z = self.int_from_bytes(z_h[0], z_l[0])/(2**14)
        qua_w =self.int_from_bytes(x_h[0], x_l[0])/(2**14)
        
        return [qua_x, qua_y, qua_z, qua_w]


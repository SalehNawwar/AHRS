#include <Wire.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

long t;
float g[3],a[3],m[3];
long count;
void setup_sensors(void) {
imu.setGyroScale(2000);  
imu.setAccelScale(16);
imu.setMagScale(16);
}
void setup() {
  Serial.begin(115200);
  Wire.begin();

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = 0x1E; // default 0x1E
  imu.settings.device.agAddress = 0x6B; // default 0x6B

  if (!imu.begin()) {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1);
  }
setup_sensors();  
  imu.calibrate(false);  // optional: bias calibration

  int mark = 200;
  Serial.write((char*)&mark,sizeof(mark));
}

void print_str(){
//  Serial.print(count);
//  Serial.print(", ");
//  Serial.println(t);
//  Serial.print(a[0],5);
//  Serial.print(", ");
//  Serial.print(a[1],5);
//  Serial.print(", ");
//  Serial.print(a[2],5);
//  Serial.print(", ");
//  Serial.print(g[0],5);
//  Serial.print(", ");
//  Serial.print(g[1],5);
//  Serial.print(", ");
//  Serial.print(g[2],5);
//  Serial.print(", ");
  Serial.print(m[0],5);
  Serial.print(", ");
  Serial.print(m[1],5);
  Serial.print(", ");
  Serial.println(m[2],5);
 // Serial.println(", ");
}

void print_bin(){
  Serial.write((char*)&count,sizeof(count));
  Serial.write((char*)&t,sizeof(t));
  Serial.write((char*)a,sizeof(a));
  Serial.write((char*)g,sizeof(g));
  Serial.write((char*)m,sizeof(m));
}
float bgx=-0.04350, bgy=-0.06089, bgz=0.00416;
void loop() {
  // تحديث البيانات
  if (imu.gyroAvailable()) imu.readGyro();
  if (imu.accelAvailable()) imu.readAccel();
  if (imu.magAvailable()) imu.readMag();
  
  // حساب الزمن dt
  t = millis();
  ++count;
  
  // السرعات الزاوية (درجة/ثانية)
  g[0] = imu.calcGyro(imu.gx);
  g[1] = imu.calcGyro(imu.gy);
  g[2] = imu.calcGyro(imu.gz);
  
  // التسارع (g)
  a[0] = imu.calcAccel(imu.ax);
  a[1] = imu.calcAccel(imu.ay);
  a[2] = imu.calcAccel(imu.az);

  m[0] = imu.calcMag(imu.mx);
  m[1] = imu.calcMag(imu.my);
  m[2] = imu.calcMag(imu.mz);
//  m[0] = 0.997*m[0] + 0.028*m[1] + 0.041*m[2];
//  m[1] = 0.028*m[0] + 1.011*m[1] + 0.041*m[2];
//  m[2] = 0.041*m[0] + 0.041*m[1] + 0.966*m[2];
  print_bin();
  delay(5);
}

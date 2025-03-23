#include <Wire.h>
#include "DFRobot_BMM150.h"

DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);

const int MAX_SAMPLES = 1500;
float samplesX[MAX_SAMPLES];
float samplesY[MAX_SAMPLES];
float samplesZ[MAX_SAMPLES];
int sampleCount = 0;

// Calibration parameters
float hardIronOffset[3] = {0};
float softIronMatrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
bool calibrating = true;

void setup()
{
  SerialUSB.begin(115200);
  Wire.begin();

  if (bmm150.begin() != BMM150_OK)
  {
    SerialUSB.println("Could not find BMM150 sensor, check wiring!");
    while (1)
      ;
  }

  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
  bmm150.setMeasurementXYZ();

  SerialUSB.println("Move sensor in all directions for 15 seconds...");
  delay(1500); // Auto-stop after 15 seconds
  calibrating = false;
  computeCalibrationParameters();
  printCalibrationParameters();
}

void loop()
{
  // Empty as calibration is done in setup
}

void computeCalibrationParameters()
{
  // Collect samples (replace with actual collection code)
  for (int i = 0; i < MAX_SAMPLES; i++)
  {
    SerialUSB.print("Collecting sample ");
    SerialUSB.print(i);
    SerialUSB.print(" of ");
    SerialUSB.println(MAX_SAMPLES);
    sBmm150MagData_t magData = bmm150.getGeomagneticData();
    samplesX[i] = magData.x;
    samplesY[i] = magData.y;
    samplesZ[i] = magData.z;
    delay(10);
  }
  sampleCount = MAX_SAMPLES;

  // Compute Hard-Iron Offset (mean)
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < sampleCount; i++)
  {
    sumX += samplesX[i];
    sumY += samplesY[i];
    sumZ += samplesZ[i];
  }
  hardIronOffset[0] = sumX / sampleCount;
  hardIronOffset[1] = sumY / sampleCount;
  hardIronOffset[2] = sumZ / sampleCount;

  // Compute centered data and covariance matrix
  float cov[6] = {0}; // xx, yy, zz, xy, xz, yz
  for (int i = 0; i < sampleCount; i++)
  {
    float dx = samplesX[i] - hardIronOffset[0];
    float dy = samplesY[i] - hardIronOffset[1];
    float dz = samplesZ[i] - hardIronOffset[2];
    cov[0] += dx * dx; // xx
    cov[1] += dy * dy; // yy
    cov[2] += dz * dz; // zz
    cov[3] += dx * dy; // xy
    cov[4] += dx * dz; // xz
    cov[5] += dy * dz; // yz
  }

  // Scale covariance by sample count
  float invN = 1.0 / sampleCount;
  for (int i = 0; i < 6; i++)
    cov[i] *= invN;

  // Compute eigenvalues and eigenvectors (simplified)
  // Note: Actual implementation requires eigenvalue decomposition
  // This is a placeholder for demonstration
  softIronMatrix[0][0] = 1 / sqrt(cov[0]);
  softIronMatrix[1][1] = 1 / sqrt(cov[1]);
  softIronMatrix[2][2] = 1 / sqrt(cov[2]);
  delay(5);
}

void printCalibrationParameters()
{
  SerialUSB.println("\n=== Calibration Complete ===");
  SerialUSB.print("Hard-Iron: ");
  SerialUSB.print(hardIronOffset[0], 4);
  SerialUSB.print(", ");
  SerialUSB.print(hardIronOffset[1], 4);
  SerialUSB.print(", ");
  SerialUSB.println(hardIronOffset[2], 4);

  SerialUSB.println("Soft-Iron Matrix:");
  for (int i = 0; i < 3; i++)
  {
    SerialUSB.print("[");
    SerialUSB.print(softIronMatrix[i][0], 4);
    SerialUSB.print(", ");
    SerialUSB.print(softIronMatrix[i][1], 4);
    SerialUSB.print(", ");
    SerialUSB.print(softIronMatrix[i][2], 4);
    SerialUSB.println("]");
  }
}
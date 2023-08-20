// #include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

// to generate more cycle, increase cycle number
// then duplicate the input based on cycle number
// for example : in_a[] = {Va[0], .... , Va[DATA_LENGTH - 1],
// Va[0], .... , Va[DATA_LENGTH - 1],
// . . .};
#define CYCLE 3
#define DATA_LENGTH 20
float Va[] = {156.63, 246.59, 294.72, 305.51, 300.66, 268.03, 204.18,
              125.41, 42.954, -48.322, -154.08, -243.95, -293.12, -303.09,
              -297.98, -264.13, -202.1, -122.25, -39.893, 51.818, 156.63, 246.59, 294.72, 305.51, 300.66, 268.03, 204.18,
              125.41, 42.954, -48.322, -154.08, -243.95, -293.12, -303.09,
              -297.98, -264.13, -202.1, -122.25, -39.893, 51.818, 156.63, 246.59, 294.72, 305.51, 300.66, 268.03, 204.18,
              125.41, 42.954, -48.322, -154.08, -243.95, -293.12, -303.09,
              -297.98, -264.13, -202.1, -122.25, -39.893, 51.818};
float Vb[] = {-308.4, -280.19, -240.66, -186.6, -99.744, -0.54547, 92.853,
              181.46, 262.05, 312.39, 311.44, 283.76, 245.04, 188.62,
              102.16, 2.9662, -89.395, -176.17, -259.16, -309.96, -308.4, -280.19, -240.66, -186.6, -99.744, -0.54547, 92.853,
              181.46, 262.05, 312.39, 311.44, 283.76, 245.04, 188.62,
              102.16, 2.9662, -89.395, -176.17, -259.16, -309.96, -308.4, -280.19, -240.66, -186.6, -99.744, -0.54547, 92.853,
              181.46, 262.05, 312.39, 311.44, 283.76, 245.04, 188.62,
              102.16, 2.9662, -89.395, -176.17, -259.16, -309.96};
float Vc[] = {156.11, 82.694, -21.783, -128.37, -213.06, -269.49, -309.58,
              -313.4, -273.73, -214.81, -154.29, -79.64, 24.679, 132.16,
              216.63, 274.14, 311.11, 315.76, 276.27, 216.22, 156.11, 82.694, -21.783, -128.37, -213.06, -269.49, -309.58,
              -313.4, -273.73, -214.81, -154.29, -79.64, 24.679, 132.16,
              216.63, 274.14, 311.11, 315.76, 276.27, 216.22, 156.11, 82.694, -21.783, -128.37, -213.06, -269.49, -309.58,
              -313.4, -273.73, -214.81, -154.29, -79.64, 24.679, 132.16,
              216.63, 274.14, 311.11, 315.76, 276.27, 216.22};

typedef struct _DDATA
{
  float *in_a;
  float va_offset;
  float va_amp;
  float *in_b;
  float *in_c;
  float *F_est;
  float *Theta_est;
  float *Harmonics;
  float Ts;
  float Kc1; // Kc are controller gains
  float Kc2; // choose your controller and
  float Kc3; // gains accordingly to get satisfied result
} DDATA;
DDATA ddata = {
    .in_a = Va,
    .in_b = Vb,
    .in_c = Vc,
    .Ts = 0.001,
};
void estimateFrequencyAndTheta(DDATA *d, int dataSize)
{
  float v_offset = 0;

  // Implementation for estimating frequency and theta
}
void getHarmonicAmplitudes(DDATA *d, int dataSize)
{
  // Implementation for getting harmonic amplitudes
}

float get_offset(float *data, int dataSize)
{
  float sum = 0;
  for (int i = 0; i < dataSize; i++)
  {
    sum += data[i];
  }

  return sum / dataSize;
}

float get_amp(float *data, int dataSize)
{
  float max = 0,
        min = 0;
  for (int i = 0; i < dataSize; i++)
  {
    max = MAX(max, data[i]);
    min = MIN(min, data[i]);
  }
  printf("max: %0.2f\r\n", max);
  printf("min: %0.2f\r\n", min);

  return (max - min) / 2;
}

int main()
{
  int i = 0;
  // printf("Va:\r\n");
  // for (i = 0; i < DATA_LENGTH * CYCLE; i++)
  // {
  //   printf("%0.2f\r\n", ddata.in_a[i]);
  // }
  // printf("Vb:\r\n");
  // for (i = 0; i < DATA_LENGTH * CYCLE; i++)
  // {
  //   printf("%0.2f\r\n", ddata.in_b[i]);
  // }
  // printf("Vc:\r\n");
  // for (i = 0; i < DATA_LENGTH * CYCLE; i++)
  // {
  //   printf("%0.2f\r\n", ddata.in_c[i]);
  // }
  printf("\r\n");

  ddata.va_offset = get_offset(&ddata.in_a[0], DATA_LENGTH * CYCLE);
  ddata.va_amp = get_amp(&ddata.in_a[0], DATA_LENGTH * CYCLE);

  printf("Va_offset: %0.2f\r\n", ddata.va_offset);
  printf("Va_amp: %0.2f\r\n", ddata.va_amp);
  // printf("Vb_offset: %0.2f\r\n", get_offset(&ddata.in_b[0], DATA_LENGTH * CYCLE));
  // printf("Vc_offset: %0.2f\r\n", get_offset(&ddata.in_c[0], DATA_LENGTH * CYCLE));

  for (i = 0; i < DATA_LENGTH * CYCLE; i++)
  {
    estimateFrequencyAndTheta(&ddata, DATA_LENGTH * CYCLE);
    getHarmonicAmplitudes(&ddata, DATA_LENGTH * CYCLE);
  }

  return 0;
}
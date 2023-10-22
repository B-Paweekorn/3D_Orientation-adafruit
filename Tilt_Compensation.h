#include "math.h"
#include <stdio.h>
typedef struct TileCompensation
{
  float rollIn;
  float pitchIn;
  float yawOut;
  float m_x, m_y, m_z;
  float XH, YH;
}TiltCom;

void TiltCom_rollpitchSetup(float roll, float pitch, TiltCom* temp);
void TiltCom_magnetoSetup(float magX, float magY, float magZ, TiltCom* temp);
void TileCom_Cal(TiltCom* temp);


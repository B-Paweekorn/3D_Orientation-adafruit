#include "Tilt_Compensation.h"

void TiltCom_rollpitchSetup(float roll, float pitch, TiltCom* temp)
{
  temp->rollIn = roll;
  temp->pitchIn = pitch;
}
void TiltCom_magnetoSetup(float magX, float magY, float magZ, TiltCom *temp)
{
  temp->m_x = magX;
  temp->m_y = magY;
  temp->m_z = magZ;
}
void TileCom_Cal(TiltCom *temp)
{
  temp->XH = temp->m_x*cos(temp->pitchIn) + temp->m_y*sin(temp->pitchIn)*sin(temp->rollIn) + temp->m_z*sin(temp->pitchIn)*cos(temp->rollIn);
  temp->YH = temp->m_y*cos(temp->rollIn) - temp->m_z*sin(temp->rollIn);
  temp->yawOut = atan2(temp->YH,temp->XH);
}

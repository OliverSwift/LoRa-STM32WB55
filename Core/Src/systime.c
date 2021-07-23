/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "systime.h"
#include "main.h"

/* Functions Definition ------------------------------------------------------*/
/**
  * @addtogroup SYSTIME_exported_function
  *  @{
  */

SysTime_t SysTimeAdd( SysTime_t a, SysTime_t b )
{
  SysTime_t c =  { .Seconds = 0, .SubSeconds = 0 };

  c.Seconds = a.Seconds + b.Seconds;
  c.SubSeconds = a.SubSeconds + b.SubSeconds;
  if( c.SubSeconds >= 1000 )
  {
    c.Seconds++;
    c.SubSeconds -= 1000;
  }
  return c;
}

SysTime_t SysTimeSub( SysTime_t a, SysTime_t b )
{
  SysTime_t c = { .Seconds = 0, .SubSeconds = 0 };

  c.Seconds = a.Seconds - b.Seconds;
  c.SubSeconds = a.SubSeconds - b.SubSeconds;
  if( c.SubSeconds < 0 )
  {
    c.Seconds--;
    c.SubSeconds += 1000;
  }
  return c;
}

void SysTimeSet( SysTime_t sysTime )
{
}

SysTime_t SysTimeGet( void )
{
  SysTime_t sysTime = { .Seconds = 0, .SubSeconds = 0 };

  uint32_t ticks = HAL_GetTick();

  sysTime.Seconds = ticks / 1000;
  sysTime.SubSeconds = ticks - sysTime.Seconds*1000;

  return sysTime;
}


SysTime_t SysTimeGetMcuTime( void )
{
  return SysTimeGet();
}

/**
  *  @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#ifndef __TYPES
#define __TYPES
  typedef signed char         sint8;
  typedef signed short int    sint16;
  typedef signed long int     sint32;
  typedef unsigned char       uint8;
  typedef unsigned short int  uint16;
  typedef unsigned long int   uint32;
  
  #ifndef NULL
    #define NULL (void*)0
  #endif
#endif
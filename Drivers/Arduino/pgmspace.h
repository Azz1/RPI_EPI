#ifndef __PGMSPACE_H_
#define __PGMSPACE_H_

#define __need_size_t
#include <inttypes.h>
#include <stddef.h>

#ifndef __ATTR_CONST__
#define __ATTR_CONST__ __attribute__((__const__))
#endif

#ifndef __ATTR_PROGMEM__
#define __ATTR_PROGMEM__ __attribute__((__progmem__))
#endif

#ifndef __ATTR_PURE__
#define __ATTR_PURE__ __attribute__((__pure__))
#endif

#ifndef flash
#define flash        const
#endif

/**
   \ingroup avr_pgmspace
   \def PROGMEM

   Attribute to use in order to declare an object being located in
   flash ROM.
*/
#define PROGMEM

#ifdef __cplusplus
extern "C" {
#endif

/**
   \ingroup avr_pgmspace
   \typedef prog_void

   Type of a "void" object located in flash ROM.  Does not make much
   sense by itself, but can be used to declare a "void *" object in
   flash ROM.
*/
typedef void PROGMEM prog_void;
/**
   \ingroup avr_pgmspace
   \typedef prog_char

   Type of a "char" object located in flash ROM.
*/
typedef char PROGMEM prog_char;

/**
   \ingroup avr_pgmspace
   \typedef prog_uchar

   Type of an "unsigned char" object located in flash ROM.
*/
typedef unsigned char PROGMEM prog_uchar;


/**
   \ingroup avr_pgmspace
   \typedef prog_int8_t

   Type of an "int8_t" object located in flash ROM.
*/
typedef int8_t PROGMEM prog_int8_t;

/**
   \ingroup avr_pgmspace
   \typedef prog_uint8_t

   Type of an "uint8_t" object located in flash ROM.
*/
typedef uint8_t PROGMEM prog_uint8_t;

/**
   \ingroup avr_pgmspace
   \typedef prog_int16_t

   Type of an "int16_t" object located in flash ROM.
*/
typedef int16_t PROGMEM prog_int16_t;

/**
   \ingroup avr_pgmspace
   \typedef prog_uint16_t

   Type of an "uint16_t" object located in flash ROM.
*/
typedef uint16_t PROGMEM prog_uint16_t;

/**
   \ingroup avr_pgmspace
   \typedef prog_int32_t

   Type of an "int32_t" object located in flash ROM.
*/
typedef int32_t PROGMEM prog_int32_t;

/**
   \ingroup avr_pgmspace
   \typedef prog_uint32_t

   Type of an "uint32_t" object located in flash ROM.
*/
typedef uint32_t PROGMEM prog_uint32_t;

/**
   \ingroup avr_pgmspace
   \typedef prog_int64_t

   Type of an "int64_t" object located in flash ROM.

   \note This type is not available when the compiler
   option -mint8 is in effect.
*/
typedef int64_t PROGMEM prog_int64_t;

/**
   \ingroup avr_pgmspace
   \typedef prog_uint64_t

   Type of an "uint64_t" object located in flash ROM.

   \note This type is not available when the compiler
   option -mint8 is in effect.
*/
typedef uint64_t PROGMEM prog_uint64_t;


/* Although in C, we can get away with just using __c, it does not work in
   C++. We need to use &__c[0] to avoid the compiler puking. Dave Hylands
   explaned it thusly,

     Let's suppose that we use PSTR("Test"). In this case, the type returned
     by __c is a prog_char[5] and not a prog_char *. While these are
     compatible, they aren't the same thing (especially in C++). The type
     returned by &__c[0] is a prog_char *, which explains why it works
     fine. */

#if defined(__DOXYGEN__)
/*
* The #define below is just a dummy that serves documentation
* purposes only.
*/
/** \ingroup avr_pgmspace
    \def PSTR(s)

    Used to declare a static pointer to a string in program space. */
# define PSTR(s) ((const PROGMEM char *)(s))
#else  /* !DOXYGEN */
/* The real thing. */
# define PSTR(s) (__extension__({static char __c[] PROGMEM = (s); &__c[0];}))
#endif /* DOXYGEN */


/** \ingroup avr_pgmspace
    \def PGM_P

    Used to declare a variable that is a pointer to a string in program
    space. */

#ifndef PGM_P
#define PGM_P const prog_char *
#endif

/** \ingroup avr_pgmspace
    \def PGM_VOID_P

    Used to declare a generic pointer to an object in program space. */

#ifndef PGM_VOID_P
#define PGM_VOID_P const prog_void *
#endif

#define pgm_read_byte(addr)       (*(unsigned char*)(addr))
#define pgm_read_word(addr)       (*(addr))//(*(const unsigned char **)(addr))
#define pgm_read_dword(p)         (*(p))
#define pgm_read_byte_near(p)     (*(unsigned char*)p)
#define pgm_read_word_near(p)     (*(p))
#define strcpy_P(dest, src)       strcpy((dest), (src))
#define strncpy_P                 strncpy
#define memcpy_P                  memcpy
#define strlen_P                  strlen
#define strcat_P                  strcat

#ifdef __cplusplus
}
#endif

#endif /* __PGMSPACE_H_ */

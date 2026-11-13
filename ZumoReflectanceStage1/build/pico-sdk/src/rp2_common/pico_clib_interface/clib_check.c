
            #include <stdio.h>
            #ifdef __PICOLIBC__
            #define USING_PICOLIBC 1
            #elif defined(__NEWLIB__)
            #define USING_NEWLIB 1
            #endif
            
            #if (picolibc && !USING_PICOLIBC) || (newlib && !USING_NEWLIB)
            #error not using wanted c library
            #endif
        
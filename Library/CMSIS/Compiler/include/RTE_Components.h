#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H

// -------- <<< Use Configuration Wizard in Context Menu >>> --------

// <o> STDOUT Interface
//    <0=> UART
//    <1=> ITM
// <i> UART: Output debug message throug defined DEBUG_PORT
// <i> ITM : Output debug message throug dedicated SWO pin
#define RTE_CMSIS_Compiler_STDOUT   0x0         /* CMSIS-Compiler STDOUT */

// <o> STDIN Interface
//    <0=> UART
// <i> UART: Output debug message throug defined DEBUG_PORT
#define RTE_CMSIS_Compiler_STDIN    0x0         /* CMSIS-Compiler STDIN */

// -------- <<< End of Configuration Wizard Context Menu >>> --------


#if (RTE_CMSIS_Compiler_STDOUT == 0)
    #define RTE_CMSIS_Compiler_STDOUT_Custom         /* CMSIS-Compiler STDOUT: Custom */
#elif (RTE_CMSIS_Compiler_STDOUT == 1)
    #define RTE_CMSIS_Compiler_STDOUT_ITM            /* CMSIS-Compiler STDOUT: ITM */
#else
    #error "Invalid STDOUT option !"
#endif

#if (RTE_CMSIS_Compiler_STDIN == 0)
    #define RTE_CMSIS_Compiler_STDIN_Custom         /* CMSIS-Compiler STDOUT: Custom */
#elif (RTE_CMSIS_Compiler_STDIN == 1)
    #define RTE_CMSIS_Compiler_STDIN_ITM            /* CMSIS-Compiler STDOUT: ITM */
#else
    #error "Invalid STDIN option !"
#endif

#endif /* RTE_COMPONENTS_H */

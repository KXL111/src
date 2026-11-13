;/*******************************************************************************************************
;**                            (asm) Copyright  苏州市江海通讯发展实业有限公司
;**                                      All Rights Reserved                                        
;**
;**---------------------------- 文件信息 ----------------------------------------------------------------
;** 文   件   名:  cstartup.s
;** 创   建   人:  林依刚
;** 版        本:  V1.1
;** 最后修改日期:  2009-8-6
;** 描        述:  增加RW,ZI的复制代码，解决了应用程序中全局变量初始化不正确的问题.
;**                完善注释.
;**---------------------------- 历史版本信息 ------------------------------------------------------------
;** 创   建   人:  林依刚
;** 版        本:  V1.0
;** 日　      期:  2007-12-10
;** 描　      述:  原始版本
;**                AT91SAM7X256汇编启动代码
;*******************************************************************************************************/

          INCLUDE   AT91SAM7X256.inc

ARM_MODE_USER       EQU       0x10
ARM_MODE_FIQ        EQU       0x11
ARM_MODE_IRQ        EQU       0x12
ARM_MODE_SVC        EQU       0x13
ARM_MODE_ABORT      EQU       0x17
ARM_MODE_UNDEF      EQU       0x1B
ARM_MODE_SYS        EQU       0x1F

I_BIT               EQU       0x80
F_BIT               EQU       0x40
          
RAM_END             EQU       0x00210000

          IMPORT    OS_CPU_IRQ_ISR

          AREA      reset, CODE, READONLY         ;/* Area Definition                                  */
          entry
;#-------------------------------------------------------------------------------------------------------
;#- Exception vectors 
;#-------------------------------------------------------------------------------------------------------
;#- These vectors can be read at address 0 or at RAM address
;#- They ABSOLUTELY requires to be in relative addresssing mode in order to guarantee a valid jump.
;#- For the moment, all are just looping.
;#- If an exception occurs before remap, this would result in an infinite loop.
;#- To ensure if a exeption occurs before start application to infinite loop.
;#-------------------------------------------------------------------------------------------------------
RESET_VEC
          B         ResetInit                     ;/* 0x00 Reset                                       */
UNDEF_VEC
          B         UNDEF_VEC                     ;/* 0x04 Undefined Instruction                       */
SWI_VEC
          B         SWI_VEC                       ;/* 0x08 Software Interrupt                          */ 
PABT_VEC 
          B         PABT_VEC                      ;/* 0x0C Prefetch Abort                              */
DABT_VEC 
          B         DABT_VEC                      ;/* 0x10 Data Abort                                  */
RESERVED_VEC
          B         RESERVED_VEC                  ;/* 0x14 Reserved                                    */
IRQ_VEC 
         ;LDR       PC, [PC, #-0xF20]             ;/* IRQ : read the AIC                               */
          LDR       PC, =OS_CPU_IRQ_ISR           ;/* Jump to μC/OS-II IRQ handler                     */
FIQ_VEC                                           ;/* 0x1c FIQ                                         */
;#-------------------------------------------------------------------------------------------------------
;#- Function             : FIQ_Handler_Entry
;#- Treatments           : FIQ Controller Interrupt Handler.
;#- Called Functions     : AIC_FVR[interrupt] 
;#-------------------------------------------------------------------------------------------------------
FIQ_Handler_Entry 
;#- Switch in SVC/User Mode to allow User Stack access for C code because the FIQ is not yet acknowledged 
;#- Save and R0 in FIQ_Register 
          MOV       R9, R0
          LDR       R0, [R8, #AIC_FVR]
          MSR       CPSR_c, #I_BIT | F_BIT | ARM_MODE_SVC

;#- Save scratch/used registers and LR in User Stack  
          STMFD     SP!, {R1-R3, R12, LR}

;#- Branch to the routine pointed by the AIC_FVR
          MOV       R14, PC
          BX        R0

;#- Restore scratch/used registers and LR from User Stack
          LDMIA     SP!, {R1-R3, R12, LR}

;#- Leave Interrupts disabled and switch back in FIQ mode
          MSR       CPSR_c, #I_BIT | F_BIT | ARM_MODE_FIQ

;#- Restore the R0 ARM_MODE_SVC register 
          MOV       R0, R9

;#- Restore the PC using the LR_fiq directly
          SUBS      PC, LR, #4

ResetInit    

          EXTERN    AT91F_LowLevelInit            ;/* c function,configure AIC,PMC...                  */
          LDR       R13, =RAM_END                 ;/* temporary stack in internal RAM                  */
               
          LDR       R0, =AT91F_LowLevelInit       ;/*Call function in ABSOLUTE through the Interwork   */
          
          MOV       LR, PC
          BX        R0
          
;#-------------------------------------------------------------------------------------------------------
;#- Stack Sizes Definition
;#-------------------------------------------------------------------------------------------------------
;#- Interrupt Stack requires 2 words x 8 priority level x 4 bytes when using the vectoring.
;#- This assume the IRQ management.Interrupt Stack must be adjusted depending on the interrupt handlers.
;#- Fast Interrupt not requires stack,If in your application it required you must be define here. 
;#- The System stack size is not defined and is limited by the free internal SRAM. 
;#-------------------------------------------------------------------------------------------------------

IRQ_STACK_SIZE      EQU       (2*8*64)            ;/* 2 words per interrupt priority level             */  

;#-------------------------------------------------------------------------------------------------------
;#- Setup the stack for each mode
;#-------------------------------------------------------------------------------------------------------
;#- Interrupt and Supervisor Stack are located at the top of internal memory in order to speed the 
;#- exception handling context saving and restoring. 
;#- ARM_MODE_SVC (Application, C) Stack is located at the top of the external memory.
;#-------------------------------------------------------------------------------------------------------
          LDR       R0, =RAM_END

;#- Enter Fast Interrupt Mode and set FIQ Mode Stack
;         MSR       CPSR_c,  #ARM_MODE_FIQ | I_BIT | F_BIT
;#- Init the FIQ register
;         LDR       R8, =AT91C_BASE_AIC

;#- Enter Interrupt mode, setup stack
          MSR       CPSR_c,  #ARM_MODE_IRQ | I_BIT | F_BIT
          MOV       R13, R0
          SUB       R0,  R0, #IRQ_STACK_SIZE

;#- Enable interrupt and enter Supervisor Mode, set Supervisor Mode Stack
          MSR       CPSR_c,  #ARM_MODE_SVC 
          MOV       R13, R0 
                             

;#-------------------------------------------------------------------------------------------------------
;#- Initialise C variables
;#-------------------------------------------------------------------------------------------------------
;#- Following labels are automatically generated by the linker. 
;#- RO: Read-only = the code
;#- RW: Read Write = the data pre-initialized and zero-initialized.
;#- ZI: Zero-Initialized.
;#- Pre-initialization values are located after the code area in the image.
;#- Zero-initialized datas are mapped after the pre-initialized.
;#- Note on the Data position : 
;#- If using the ARMSDT, when no -rw-base option is used for the linker, the data area is mapped after the code.
;#- You can map the data either in internal SRAM ( -rw-base=0x40 or 0x34) or in external SRAM ( -rw-base=0x2000000 ). 
;#- Note also that to improve the code density, the pre_initialized data must be limited to a minimum. 
;#-------------------------------------------------------------------------------------------------------         
          IMPORT    |Image$$RO$$Base|             ;/* Base and limit of ROM code (=start of ROM data)  */
          IMPORT    |Image$$RO$$Limit|            
          IMPORT    |Image$$RW$$Base|             ;/* Base and limit of RAM to pre-initialize          */
          IMPORT    |Image$$RW$$Limit|
          IMPORT    |Image$$ZI$$Base|             ;/* Base and limit of area to zero initialize        */
          IMPORT    |Image$$ZI$$Limit|

          LDR       R0, =|Image$$RO$$Limit|       ;/* Get start pointer neading copy form ROM to RAM   */
          LDR       R1, =|Image$$RW$$Base|
          LDR       R2, =|Image$$ZI$$Base|        ;/* Zero init base => top of initialized data        */
COPY_RW       
          CMP       R1, R2                        ;/* Copy init data                                   */
          LDRLO     R3, [R0], #4
          STRLO     R3, [R1], #4
          BLO       COPY_RW
           
          MOV       R3, #0
          LDR       R1, =|Image$$ZI$$Limit|       ;/* Top of zero init segment                         */
COPY_ZI
          CMP       R2, R1                        ;/* Zero init                                        */
          STRLO     R3, [R2], #4
          BLO       COPY_ZI  

                          
;#-------------------------------------------------------------------------------------------------------
;#- Enter in the main function
;#-------------------------------------------------------------------------------------------------------
          EXTERN    main
          global    __main
jump_to_main
          LDR	    LR, =call_exit
          LDR	    R0, =main
__main
          BX	    R0

;#-------------------------------------------------------------------------------------------------------
;#- Loop for ever, End of application. Normally, never occur.Could jump to software reset (B 0x00000000). 
;#-------------------------------------------------------------------------------------------------------
call_exit
End
          B         End

;#-------------------------------------------------------------------------------------------------------
;#- Function             : IRQ_Handler_Entry
;#- Treatments           : IRQ Controller Interrupt Handler.
;#- Called Functions     : AIC_IVR[interrupt] 
;#-------------------------------------------------------------------------------------------------------
IRQ_Handler_Entry 
;#- Manage Exception Entry,adjust and save LR_irq in IRQ stack
          SUB       LR,  LR, #4
          STMFD     SP!, {LR}
          
;#- Save and R0 in IRQ stack
          STMFD     SP!, {R0}

;#- Write in the IVR to support Protect Mode
;#- No effect in Normal Mode
;#- De-assert the NIRQ and clear the source in Protect Mode
          LDR       R14, =AT91C_BASE_AIC
          LDR       R0,  [R14, #AIC_IVR]
          STR       R14, [R14, #AIC_IVR]

;#- Enable Interrupt and Switch in Supervisor Mode
          MSR       CPSR_c, #ARM_MODE_SVC

;#- Save scratch/used registers and LR in User Stack
          STMFD     SP!, {R1-R3, R12, R14}

;#- Branch to the routine pointed by the AIC_IVR
          MOV       R14, PC
          BX        R0

;#- Restore scratch/used registers and LR from User Stack
          LDMIA     SP!, {R1-R3, R12, R14}

;#- Disable Interrupt and switch back in IRQ mode
          MSR       CPSR_c, #I_BIT | ARM_MODE_IRQ

;#- Mark the End of Interrupt on the AIC
          LDR       R14, =AT91C_BASE_AIC
          STR       R14, [R14, #AIC_EOICR]

;#- Restore SPSR_irq and R0 from IRQ stack
          LDMIA     SP!, {R0}

;#- Restore adjusted  LR_irq from IRQ stack directly in the PC
          LDMIA     SP!, {PC}^
            
;#-------------------------------------------------------------------------------------------------------
;# The default interrupt serve routine for the CPU
;#-------------------------------------------------------------------------------------------------------
          global	AT91F_DefaultFIQHandler
          global	AT91F_DefaultIRQHandler
          global	AT91F_SpuriousHandler

AT91F_DefaultFIQHandler
          B         AT91F_DefaultFIQHandler

AT91F_DefaultIRQHandler
          B         AT91F_DefaultIRQHandler

AT91F_SpuriousHandler
          B         AT91F_SpuriousHandler
          
	      END

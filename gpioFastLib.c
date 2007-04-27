/*
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

/*
 * This program is written By HardElettroSoft Team. Thanks to Giuseppe Manzoni.
 *
 * It is based on the work of Roberto Asquini for its FastIO_C_example.
 *  FastIO_C_example is based on the work of Douglas Gilbert for its mem2io.c
 *  for accessing input output register of the CPU from userspace
 *
 * Changelog - Hardlettrosoft
 * ---------------------------------
 * 2013 April 01 - First release
 * 2013 April 05 - Update comments and text formatting
 * 2013 April 08 - Update function name readGpio(..) in fastReadGpio(..)
 * 2016 Febrary 08 - Update with others registers, move verbose check form runtime to compiler task
 *
 */

// uncomment if you want verbose for debug
// #define verbose


#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <math.h>

#define DEV_MEM "/dev/mem"
#define MAP_SIZE 4096   /* needs to be a power of 2! */
#define MAP_MASK (MAP_SIZE - 1)

// addresses of interesting physical Port A registers
#define PIOA_OER  0xfffff410   // (Wr) PIO Output Enable Register -> 1 to the bit that has to be put in output
#define PIOA_ODR  0xfffff414   // (Wr) PIO Output Disable Register -> 1 to the bit that has to be put in input
#define PIOA_SODR 0xfffff430   // (Wr) PIO Set Output Data Register -> 1 to the output bit that has to be set
#define PIOA_CODR 0xfffff434   // (Wr) PIO Clear Output Data Register -> 1 to the output bit that has to be cleared
#define PIOA_ODSR 0xfffff438   // (Rd) PIO Output Data Status Register : to read the output status of the PortA pins
#define PIOA_PDSR 0xfffff43C   // (Rd) PIO Pin Data Status Register _ to read the status of the PortA input pins

// addresses of interesting physical Port B registers
#define PIOB_OER  0xfffff610   // (Wr) PIO Output Enable Register -> 1 to the bit that has to be put in output
#define PIOB_ODR  0xfffff614   // (Wr) PIO Output Disable Register -> 1 to the bit that has to be put in input
#define PIOB_SODR 0xfffff630   // (Wr) PIO Set Output Data Register -> 1 to the output bit that has to be set
#define PIOB_CODR 0xfffff634   // (Wr) PIO Clear Output Data Register -> 1 to the output bit that has to be cleared
#define PIOB_ODSR 0xfffff638   // (Rd) PIO Output Data Status Register : to read the output status of the PortB pins
#define PIOB_PDSR 0xfffff63C   // (Rd) PIO Pin Data Status Register _ to read the status of the PortB input pins

// addresses of interesting physical Port C registers
#define PIOC_OER  0xfffff810   // (Wr) PIO Output Enable Register -> 1 to the bit that has to be put in output
#define PIOC_ODR  0xfffff814   // (Wr) PIO Output Disable Register -> 1 to the bit that has to be put in input
#define PIOC_SODR 0xfffff830   // (Wr) PIO Set Output Data Register -> 1 to the output bit that has to be set
#define PIOC_CODR 0xfffff834   // (Wr) PIO Clear Output Data Register -> 1 to the output bit that has to be cleared
#define PIOC_ODSR 0xfffff838   // (Rd) PIO Output Data Status Register : to read the output status of the PortC pins
#define PIOC_PDSR 0xfffff83C   // (Rd) PIO Pin Data Status Register _ to read the status of the PortC input pins

// addresses of interesting physical Port D registers
#define PIOD_OER  0xfffffA10   // (Wr) PIO Output Enable Register -> 1 to the bit that has to be put in output
#define PIOD_ODR  0xfffffA14   // (Wr) PIO Output Disable Register -> 1 to the bit that has to be put in input
#define PIOD_SODR 0xfffffA30   // (Wr) PIO Set Output Data Register -> 1 to the output bit that has to be set
#define PIOD_CODR 0xfffffA34   // (Wr) PIO Clear Output Data Register -> 1 to the output bit that has to be cleared
#define PIOD_ODSR 0xfffffA38   // (Rd) PIO Output Data Status Register : to read the output status of the PortD pins
#define PIOD_PDSR 0xfffffA3C   // (Rd) PIO Pin Data Status Register _ to read the status of the PortD input pins



int mem_fd;
void * mmap_ptr;
off_t mask_addr;

// variables to store the mapped address of the interesting registers
void * mapped_PIOA_OER_addr;
void * mapped_PIOA_ODR_addr;
void * mapped_PIOA_SODR_addr;
void * mapped_PIOA_CODR_addr;
void * mapped_PIOA_ODSR_addr;
void * mapped_PIOA_PDSR_addr;

void * mapped_PIOB_OER_addr;
void * mapped_PIOB_ODR_addr;
void * mapped_PIOB_SODR_addr;
void * mapped_PIOB_CODR_addr;
void * mapped_PIOB_ODSR_addr;
void * mapped_PIOB_PDSR_addr;

void * mapped_PIOC_OER_addr;
void * mapped_PIOC_ODR_addr;
void * mapped_PIOC_SODR_addr;
void * mapped_PIOC_CODR_addr;
void * mapped_PIOC_ODSR_addr;
void * mapped_PIOC_PDSR_addr;

void * mapped_PIOD_OER_addr;
void * mapped_PIOD_ODR_addr;
void * mapped_PIOD_SODR_addr;
void * mapped_PIOD_CODR_addr;
void * mapped_PIOD_ODSR_addr;
void * mapped_PIOD_PDSR_addr;


unsigned int init_memoryToIO(void);
unsigned int close_memoryToIO(void);
void setPortAinInput(void);
void setPortBinInput(void);
void setPortCinInput(void);
void setPortDinInput(void);
void setPortAinOutput(void);
void setPortBinOutput(void);
void setPortCinOutput(void);
void setPortDinOutput(void);
unsigned int readGeneralRegister(unsigned int reg);
unsigned int readPortAoutbits(void);
unsigned int readPortBoutbits(void);
unsigned int readPortCoutbits(void);
unsigned int readPortDoutbits(void);
unsigned int readPortAinbits(void);
unsigned int readPortBinbits(void);
unsigned int readPortCinbits(void);
unsigned int readPortDinbits(void);
void writePortA(unsigned int uintData);
void writePortB(unsigned int uintData);
void writePortC(unsigned int uintData);
void writePortD(unsigned int uintData);
void setGpioAinInput(unsigned int uintGpio);
void setGpioBinInput(unsigned int uintGpio);
void setGpioCinInput(unsigned int uintGpio);
void setGpioDinInput(unsigned int uintGpio);
void setGpioAinOutput(unsigned int uintGpio);
void setGpioBinOutput(unsigned int uintGpio);
void setGpioCinOutput(unsigned int uintGpio);
void setGpioDinOutput(unsigned int uintGpio);
void fastSetGpioA(unsigned int uintGpio);
void fastSetGpioB(unsigned int uintGpio);
void fastSetGpioC(unsigned int uintGpio);
void fastSetGpioD(unsigned int uintGpio);
void fastClearGpioA(unsigned int uintGpio);
void fastClearGpioB(unsigned int uintGpio);
void fastClearGpioC(unsigned int uintGpio);
void fastClearGpioD(unsigned int uintGpio);
unsigned int fastReadGpioA(unsigned int uintGpio);
unsigned int fastReadGpioB(unsigned int uintGpio);
unsigned int fastReadGpioC(unsigned int uintGpio);
unsigned int fastReadGpioD(unsigned int uintGpio);
// optimized examples
void fastSetPC3(void);
void fastClearPC3(void);


// to map in a local page the peripheral address registers used
unsigned int init_memoryToIO(void) {
    mem_fd = -1;

    if ((mem_fd = open(DEV_MEM, O_RDWR | O_SYNC)) < 0) {
        printf("open of " DEV_MEM " failed");
        return 1;
    }
#ifdef verbose
	else printf("open(" DEV_MEM "O_RDWR | O_SYNC) okay\n");
#endif
    mask_addr = (PIOA_OER & ~MAP_MASK);  // preparation of mask_addr (base of the memory accessed)
    mask_addr = (PIOB_OER & ~MAP_MASK);  // preparation of mask_addr (base of the memory accessed)
    mask_addr = (PIOC_OER & ~MAP_MASK);  // preparation of mask_addr (base of the memory accessed)
    mask_addr = (PIOD_OER & ~MAP_MASK);  // preparation of mask_addr (base of the memory accessed)

#ifdef verbose
    printf ("Mask address = %08x\n",mask_addr);
#endif
    mmap_ptr = (void *)-1;
    mmap_ptr = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE,
                       MAP_SHARED, mem_fd, mask_addr);
#ifdef verbose
    printf ("Mmap_ptr = %08x\n",mmap_ptr);
#endif
    if ((void *)-1 == mmap_ptr) {
        printf("addr=0x%x, mask_addr=0x%lx :\n", PIOA_OER, mask_addr);
        printf("    mmap\n\r");
        printf("addr=0x%x, mask_addr=0x%lx :\n", PIOB_OER, mask_addr);
        printf("    mmap\n\r");
        printf("addr=0x%x, mask_addr=0x%lx :\n", PIOC_OER, mask_addr);
        printf("    mmap\n\r");
        printf("addr=0x%x, mask_addr=0x%lx :\n", PIOD_OER, mask_addr);
        printf("    mmap\n\r");
        return 1;
    }
#ifdef verbose
    printf("mmap() ok, mask_addr=0x%lx, mmap_ptr=%p\n", mask_addr, mmap_ptr);
#endif
    // PORTA
    mapped_PIOA_OER_addr = mmap_ptr + (PIOC_OER & MAP_MASK);
    mapped_PIOA_ODR_addr = mmap_ptr + (PIOC_ODR & MAP_MASK);
    mapped_PIOA_SODR_addr = mmap_ptr + (PIOC_SODR & MAP_MASK);
    mapped_PIOA_CODR_addr = mmap_ptr + (PIOC_CODR & MAP_MASK);
    mapped_PIOA_ODSR_addr = mmap_ptr + (PIOC_ODSR & MAP_MASK);
    mapped_PIOA_PDSR_addr = mmap_ptr + (PIOC_PDSR & MAP_MASK);

    // PORTB
    mapped_PIOB_OER_addr = mmap_ptr + (PIOC_OER & MAP_MASK);
    mapped_PIOB_ODR_addr = mmap_ptr + (PIOC_ODR & MAP_MASK);
    mapped_PIOB_SODR_addr = mmap_ptr + (PIOC_SODR & MAP_MASK);
    mapped_PIOB_CODR_addr = mmap_ptr + (PIOC_CODR & MAP_MASK);
    mapped_PIOB_ODSR_addr = mmap_ptr + (PIOC_ODSR & MAP_MASK);
    mapped_PIOB_PDSR_addr = mmap_ptr + (PIOC_PDSR & MAP_MASK);

    // PORTC
    mapped_PIOC_OER_addr = mmap_ptr + (PIOC_OER & MAP_MASK);
    mapped_PIOC_ODR_addr = mmap_ptr + (PIOC_ODR & MAP_MASK);
    mapped_PIOC_SODR_addr = mmap_ptr + (PIOC_SODR & MAP_MASK);
    mapped_PIOC_CODR_addr = mmap_ptr + (PIOC_CODR & MAP_MASK);
    mapped_PIOC_ODSR_addr = mmap_ptr + (PIOC_ODSR & MAP_MASK);
    mapped_PIOC_PDSR_addr = mmap_ptr + (PIOC_PDSR & MAP_MASK);

    // PORTD
    mapped_PIOD_OER_addr = mmap_ptr + (PIOC_OER & MAP_MASK);
    mapped_PIOD_ODR_addr = mmap_ptr + (PIOC_ODR & MAP_MASK);
    mapped_PIOD_SODR_addr = mmap_ptr + (PIOC_SODR & MAP_MASK);
    mapped_PIOD_CODR_addr = mmap_ptr + (PIOC_CODR & MAP_MASK);
    mapped_PIOD_ODSR_addr = mmap_ptr + (PIOC_ODSR & MAP_MASK);
    mapped_PIOD_PDSR_addr = mmap_ptr + (PIOC_PDSR & MAP_MASK);

    return 0;
}


// closing memory mapping
unsigned int close_memoryToIO(void) {
    if (-1 == munmap(mmap_ptr, MAP_SIZE)) {
        printf("mmap_ptr=%p:\n", mmap_ptr);
        printf("    munmap");
        return 1;
    }
#ifdef verbose
    else printf("call of munmap() ok, mmap_ptr=%p\n", mmap_ptr);
#endif
    if (mem_fd >= 0) close(mem_fd);
    return 0;
}


// put PortX in input mode
void setPortAinInput(void) { *((unsigned long *)mapped_PIOA_ODR_addr) = 0xffffffff;}
void setPortBinInput(void) { *((unsigned long *)mapped_PIOB_ODR_addr) = 0xffffffff;}
void setPortCinInput(void) { *((unsigned long *)mapped_PIOC_ODR_addr) = 0xffffffff;}
void setPortDinInput(void) { *((unsigned long *)mapped_PIOD_ODR_addr) = 0xffffffff;}

// put PortX in output mode
void setPortAinOutput(void) { *((unsigned long *)mapped_PIOA_OER_addr) = 0xffffffff;}
void setPortBinOutput(void) { *((unsigned long *)mapped_PIOB_OER_addr) = 0xffffffff;}
void setPortCinOutput(void) { *((unsigned long *)mapped_PIOC_OER_addr) = 0xffffffff;}
void setPortDinOutput(void) { *((unsigned long *)mapped_PIOD_OER_addr) = 0xffffffff;}


unsigned int readGeneralRegister(unsigned int reg) {
    void * ap;
    unsigned long ul; // returns the content of the CPU register reg

    ap = mmap_ptr + (reg & MAP_MASK);
    ul = *((unsigned long *)ap);    // read the register
#ifdef verbose
    printf("read: addr=0x%x, val=0x%x\n", reg, (unsigned int)ul);
#endif
    return (unsigned int)ul;
}


unsigned int readPortAoutbits(void) {
    unsigned long ul;    // returns the content of the register reg

    ul = *((unsigned long *)mapped_PIOA_ODSR_addr);    // read the register
#ifdef verbose
    printf("read: addr=0x%x, val=0x%x\n", PIOA_ODSR, (unsigned int)ul);
#endif
    return (unsigned int)ul;
}
unsigned int readPortBoutbits(void) {
    unsigned long ul;    // returns the content of the register reg

    ul = *((unsigned long *)mapped_PIOB_ODSR_addr);    // read the register
#ifdef verbose
    printf("read: addr=0x%x, val=0x%x\n", PIOB_ODSR, (unsigned int)ul);
#endif
    return (unsigned int)ul;
}
unsigned int readPortCoutbits(void) {
    unsigned long ul;    // returns the content of the register reg

    ul = *((unsigned long *)mapped_PIOC_ODSR_addr);    // read the register
#ifdef verbose
    printf("read: addr=0x%x, val=0x%x\n", PIOC_ODSR, (unsigned int)ul);
#endif
    return (unsigned int)ul;
}
unsigned int readPortDoutbits(void) {
    unsigned long ul;    // returns the content of the register reg

    ul = *((unsigned long *)mapped_PIOD_ODSR_addr);    // read the register
#ifdef verbose
    printf("read: addr=0x%x, val=0x%x\n", PIOD_ODSR, (unsigned int)ul);
#endif
    return (unsigned int)ul;
}


unsigned int readPortAinbits(void) {
    unsigned long ul; // returns the content of the register reg

    ul = *((unsigned long *)mapped_PIOA_PDSR_addr);     // read the register
#ifdef verbose
    printf("read: addr=0x%x, val=0x%x\n", PIOA_PDSR, (unsigned int)ul);
#endif
    return (unsigned int)ul;
}
unsigned int readPortBinbits(void) {
    unsigned long ul; // returns the content of the register reg

    ul = *((unsigned long *)mapped_PIOB_PDSR_addr);     // read the register
#ifdef verbose
    printf("read: addr=0x%x, val=0x%x\n", PIOB_PDSR, (unsigned int)ul);
#endif
    return (unsigned int)ul;
}
unsigned int readPortCinbits(void) {
    unsigned long ul; // returns the content of the register reg

    ul = *((unsigned long *)mapped_PIOC_PDSR_addr);     // read the register
#ifdef verbose
    printf("read: addr=0x%x, val=0x%x\n", PIOC_PDSR, (unsigned int)ul);
#endif
    return (unsigned int)ul;
}
unsigned int readPortDinbits(void) {
    unsigned long ul; // returns the content of the register reg

    ul = *((unsigned long *)mapped_PIOD_PDSR_addr);     // read the register
#ifdef verbose
    printf("read: addr=0x%x, val=0x%x\n", PIOD_PDSR, (unsigned int)ul);
#endif
    return (unsigned int)ul;
}


// write the output registers of Port X with the value "data"
void writePortA(unsigned int uintData) {
    *((unsigned long *)mapped_PIOA_SODR_addr) = uintData;
    *((unsigned long *)mapped_PIOA_CODR_addr) = ~uintData;
}
void writePortB(unsigned int uintData) {
    *((unsigned long *)mapped_PIOB_SODR_addr) = uintData;
    *((unsigned long *)mapped_PIOB_CODR_addr) = ~uintData;
}
void writePortC(unsigned int uintData) {
    *((unsigned long *)mapped_PIOC_SODR_addr) = uintData;
    *((unsigned long *)mapped_PIOC_CODR_addr) = ~uintData;
}
void writePortD(unsigned int uintData) {
    *((unsigned long *)mapped_PIOD_SODR_addr) = uintData;
    *((unsigned long *)mapped_PIOD_CODR_addr) = ~uintData;
}


/*
Function:       void setGpioXinInput(unsigned int uintGpio)
Aim:            Put given gpio in input mode
Parameters:
		uintGpio -> Port X bit
Return:         -
Revision:       1 - First Release
Author:         HardElettroSoft Team, Giovanni Manzoni
Note:           -
*/
void setGpioAinInput(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOA_ODR_addr) = ulnGpio;
}
void setGpioBinInput(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOB_ODR_addr) = ulnGpio;
}
void setGpioCinInput(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOC_ODR_addr) = ulnGpio;
}
void setGpioDinInput(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOD_ODR_addr) = ulnGpio;
}


/*
Function:       void setGpioXinOutput(unsigned int uintGpio)
Aim:            Put given gpio in output mode
Parameters:
		uintGpio -> Port X bit
Return:         -
Revision:       1 - First Release
Author:		HardElettroSoft Team, Giovanni Manzoni
Note:           -
*/
void setGpioAinOutput(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOA_OER_addr) = ulnGpio;
}
void setGpioBinOutput(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOB_OER_addr) = ulnGpio;
}
void setGpioCinOutput(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOC_OER_addr) = ulnGpio;
}
void setGpioDinOutput(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOD_OER_addr) = ulnGpio;
}


/*
Function:       void fastSetGpioX(unsigned int uintGpio)
Aim:            Set given gpio
Parameters:
		uintGpio -> Port X bit
Return:         -
Revision:       1 - First Release
Author:		HardElettroSoft Team, Giovanni Manzoni
Note:           -
*/
void fastSetGpioA(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOA_SODR_addr) = ulnGpio;
}
void fastSetGpioB(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOB_SODR_addr) = ulnGpio;
}
void fastSetGpioC(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOC_SODR_addr) = ulnGpio;
}
void fastSetGpioD(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOD_SODR_addr) = ulnGpio;
}


/*
Function:       void fastClearGpioX(unsigned int uintGpio)
Aim:            Clear given gpio
Parameters:
		uintGpio -> Port X bit
Return:         -
Revision:       1 - First Release
Author:		HardElettroSoft Team, Giovanni Manzoni
Note:           -
*/
void fastClearGpioA(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOA_CODR_addr) = ulnGpio;
}
void fastClearGpioB(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOB_CODR_addr) = ulnGpio;
}
void fastClearGpioC(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOC_CODR_addr) = ulnGpio;
}
void fastClearGpioD(unsigned int uintGpio) {
    unsigned long ulnGpio = 1 << uintGpio;
    *((unsigned long *)mapped_PIOD_CODR_addr) = ulnGpio;
}

/*
Function: 	unsigned int fastReadGpioX(unsigned int uintGpio)
Aim:		read logic level of given gpio
Parameters:
                uintGpio -> Port X bit
Return: 	logic level
Revision: 	1 - First Release
Author:		HardElettroSoft
Note: 		-
*/
unsigned int fastReadGpioA(unsigned int uintGpio){
    unsigned long ul; // returns the content of the register reg
    unsigned int uintValue;
    ul = *((unsigned long *)mapped_PIOA_PDSR_addr);     // read the register
    uintValue = ( ul & (1 << uintGpio)) >> uintGpio;  // mask and get single bit value
    return uintValue;
}
unsigned int fastReadGpioB(unsigned int uintGpio){
    unsigned long ul; // returns the content of the register reg
    unsigned int uintValue;
    ul = *((unsigned long *)mapped_PIOB_PDSR_addr);     // read the register
    uintValue = ( ul & (1 << uintGpio)) >> uintGpio;  // mask and get single bit value
    return uintValue;
}
unsigned int fastReadGpioC(unsigned int uintGpio){
    unsigned long ul; // returns the content of the register reg
    unsigned int uintValue;
    ul = *((unsigned long *)mapped_PIOC_PDSR_addr);     // read the register
    uintValue = ( ul & (1 << uintGpio)) >> uintGpio;  // mask and get single bit value
    return uintValue;
}
unsigned int fastReadGpioD(unsigned int uintGpio){
    unsigned long ul; // returns the content of the register reg
    unsigned int uintValue;
    ul = *((unsigned long *)mapped_PIOD_PDSR_addr);     // read the register
    uintValue = ( ul & (1 << uintGpio)) >> uintGpio;  // mask and get single bit value
    return uintValue;
}


/*
Function:       void fastSetPC3(void)
Aim:            Set PC3 = N5 = Kernel ID 99
Parameters:     -
Return:         -
Revision:       1 - First Release
Author:		HardElettroSoft Team, Giuseppe Manzoni
Note:           It is not really more faster than fastSetGpio(..) but a bit more
*/
void fastSetPC3(void){
    *((unsigned long *)mapped_PIOC_SODR_addr) = 8;
}


/*
Function: 	void fastClearPC3(void)
Aim:		Clear PC3 = N5 = Kernel ID 99
Parameters: 	-
Return: 	-
Revision: 	1 - First Release
Author:		HardElettroSoft Team, Giuseppe Manzoni
Note: 		It is not really more faster than fastClearGpio(..)
*/
void fastClearPC3(void){
    *((unsigned long *)mapped_PIOC_CODR_addr) = 8;
}

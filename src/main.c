// NOTE!!: PA0 actually connects to IN5 of ADC1.
//
#include <stm32l432xx.h>
#include <stdint.h>
#include <stdio.h>
#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
void setup(void);
void delay(volatile uint32_t dly);
void enablePullUp(GPIO_TypeDef *Port, uint32_t BitNumber);
void pinMode(GPIO_TypeDef *Port, uint32_t BitNumber, uint32_t Mode);
void initClocks(); // Needed this
void selectAlternateFunction (GPIO_TypeDef *Port, uint32_t BitNumber, uint32_t AF); // Need this
void initSerial(uint32_t baudrate); // Need this
void eputc(char c); // Need this
int buttonPressed(void);
u_int32_t milliseconds = 0;
u_int32_t millis();
int main()
{
    setup();
    int count = 0;
    int currentButton = 0;
    int previousButton = 0;
    SystemCoreClockUpdate();
    uint32_t clk = SystemCoreClock;
    
    SysTick->LOAD = 80000-1; // Systick clock = 80MHz. 80000000/44100 = 1814
	SysTick->CTRL = 7; // enable systick counter and its interrupts
	SysTick->VAL = 10; // start from a low number so we don't wait for ages for first interrupt
	
    __asm(" cpsie i "); // enable interrupts globally

    while(1)
    {
        printf("System clock is %ld \r\n",clk);

        currentButton = buttonPressed();    // Get curretn state of button press
        
        if (previousButton == 0 && currentButton == 1) // Only enter the if statement if there is a rising edge
        {
            GPIOB->ODR |= (1 << 3);     // Turn on the LED
            count++;
            printf("Button count: %d\r\n",count);

            // Enter LP run mode
            // Set Bit 13 of FLASH_ACR (write protecte see ref manual)
            FLASH->PDKEYR = 0x04152637;
            FLASH->PDKEYR = 0xFAFBFCFD; 
            FLASH->ACR |= (1 << 13); 
            
            // Decrease system clock to below 2 Mhz

            // Turn off PLL first? Not required.
            RCC->CR &= ~(1 << 24); // Turn off PLL

            // Enable internal oscillator for MSI
            RCC->CR |= (1 << 0);            // Enable MSI (multi-speed internal oscillator)
            while(!(RCC->CR & (1 << 1)));   // Wait till MSI ready bit is set
            
            // Configure MSI clock to 1 MHz
            RCC->CR |= (1 << 3);        // Allows MSI clock range to set by MSIRANGW bits in CR register
            RCC->CR &= ~(0b1111 << 4);  // Clear the MSIRANGE bits 
            RCC->CR |= (3 << 4);        // Set MSIRANGE to mode 4 (1 MHz)

            // Select MSI as system clock
            RCC->CFGR &= ~(0b11 << 0);      // Clear system clock switch bits
            RCC->CFGR |= (0b00 << 0);       // Set system switch bits (SW bits, 00: MSI as system clock)

            while( (RCC->CFGR & (0b11 << 2)) != 0); // Wait until MSI is set as system clock
            
            // Update system clock
            SystemCoreClockUpdate();
            clk = SystemCoreClock;

            // Force regulator into low-power mode (PWR->CR1, Bit 14), requires system clokc to be < 2 MHz
            PWR->CR1 |= (1u << 14);

            
            initSerial(9600);
            
            

        }   
        else
        {
            GPIOB->ODR &= ~(1 << 3);    // Turns off the LED
        }  
        
        previousButton = currentButton;     // Store previous state

    }
}
void setup(void)
{
    initClocks();
    RCC->AHB2ENR |= (1 << 0) | (1 << 1); // turn on GPIOA and GPIOB
    initSerial(9600);
    pinMode(GPIOB,3,1);
    pinMode(GPIOB,4,0);
    enablePullUp(GPIOB,4);
}
void delay(volatile uint32_t dly)
{
    while(dly--);
}
void enablePullUp(GPIO_TypeDef *Port, uint32_t BitNumber)
{
	Port->PUPDR = Port->PUPDR &~(3u << BitNumber*2); // clear pull-up resistor bits
	Port->PUPDR = Port->PUPDR | (1u << BitNumber*2); // set pull-up bit
}
void pinMode(GPIO_TypeDef *Port, uint32_t BitNumber, uint32_t Mode)
{
	/*
        Modes : 00 = input
                01 = output
                10 = special function
                11 = analog mode
	*/
	uint32_t mode_value = Port->MODER;
	Mode = Mode << (2 * BitNumber);
	mode_value = mode_value & ~(3u << (BitNumber * 2));
	mode_value = mode_value | Mode;
	Port->MODER = mode_value;
}
void eputc(char c)
{
    while( (USART2->ISR & (1 << 6))==0); // wait for ongoing transmission to finish
    USART2->TDR=c;
}
int _write(int file, char *data, int len)
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
    {
        errno = EBADF;
        return -1;
    }
    while(len--)
    {
        while( (USART2->ISR & (1 << 6))==0); // wait for ongoing transmission to finish
        USART2->TDR=*data;    
        data++;
    }    
    return 0;
}
void initSerial(uint32_t baudrate)
{
    RCC->AHB2ENR |= (1 << 0); // make sure GPIOA is turned on
    pinMode(GPIOA,2,2); // alternate function mode for PA2
    selectAlternateFunction(GPIOA,2,7); // AF7 = USART2 TX

    pinMode(GPIOA,15,2); // alternate function mode for PA15
    selectAlternateFunction(GPIOA,15,3); // AF3 = USART2 RX


    RCC->APB1ENR1 |= (1 << 17); // turn on USART2

    SystemCoreClockUpdate();
	// const uint32_t CLOCK_SPEED=80000000;
    const uint32_t CLOCK_SPEED=SystemCoreClock;
	uint32_t BaudRateDivisor;
	
	BaudRateDivisor = CLOCK_SPEED/baudrate;	
	USART2->CR1 = 0;
	USART2->CR2 = 0;
	USART2->CR3 = (1 << 12); // disable over-run errors
	USART2->BRR = BaudRateDivisor;
	USART2->CR1 =  (1 << 3) | (1 << 2);  // enable the transmitter and receiver
    USART2->CR1 |= (1 << 5); // enable receiver interrupts
    USART2->CR3 = (1 << 12); // disable over-run errors
	USART2->CR1 |= (1 << 0); // enable the UART
    USART2->ICR = (1 << 1); // clear any old framing errors
    
}
void initClocks()
{
	// Initialize the clock system to a higher speed.
	// At boot time, the clock is derived from the MSI clock 
	// which defaults to 4MHz.  Will set it to 80MHz
	// See chapter 6 of the reference manual (RM0393)
	    RCC->CR &= ~(1 << 24); // Make sure PLL is off
	
	// PLL Input clock = MSI so BIT1 = 1, BIT 0 = 0
	// PLLM = Divisor for input clock : set = 1 so BIT6,5,4 = 0
	// PLL-VCO speed = PLL_N x PLL Input clock
	// This must be < 344MHz
	// PLL Input clock = 4MHz from MSI
	// PLL_N can range from 8 to 86.  
	// Will use 80 for PLL_N as 80 * 4 = 320MHz
	// Put value 80 into bits 14:8 (being sure to clear bits as necessary)
	// PLLSAI3 : Serial audio interface : not using leave BIT16 = 0
	// PLLP : Must pick a value that divides 320MHz down to <= 80MHz
	// If BIT17 = 1 then divisor is 17; 320/17 = 18.82MHz : ok (PLLP used by SAI)
	// PLLQEN : Don't need this so set BIT20 = 0
	// PLLQ : Must divide 320 down to value <=80MHz.  
	// Set BIT22,21 to 1 to get a divisor of 8 : ok
	// PLLREN : This enables the PLLCLK output of the PLL
	// I think we need this so set to 1. BIT24 = 1 
	// PLLR : Pick a value that divides 320 down to <= 80MHz
	// Choose 4 to give an 80MHz output.  
	// BIT26 = 0; BIT25 = 1
	// All other bits reserved and zero at reset
	    RCC->PLLCFGR = (1 << 25) + (1 << 24) + (1 << 22) + (1 << 21) + (1 << 17) + (80 << 8) + (1 << 0);	
	    RCC->CR |= (1 << 24); // Turn PLL on
	    while( (RCC->CR & (1 << 25))== 0); // Wait for PLL to be ready
	// configure flash for 4 wait states (required at 80MHz)
	    FLASH->ACR &= ~((1 << 2)+ (1 << 1) + (1 << 0));
	    FLASH->ACR |= (1 << 2); 
	    RCC->CFGR |= (1 << 1)+(1 << 0); // Select PLL as system clock
}
void selectAlternateFunction (GPIO_TypeDef *Port, uint32_t BitNumber, uint32_t AF)
{
    // The alternative function control is spread across two 32 bit registers AFR[0] and AFR[1]
    // There are 4 bits for each port bit.
    if (BitNumber < 8)
    {
        Port->AFR[0] &= ~(0x0f << (4*BitNumber));
        Port->AFR[0] |= (AF << (4*BitNumber));
    }
    else
    {
        BitNumber = BitNumber - 8;
        Port->AFR[1] &= ~(0x0f << (4*BitNumber));
        Port->AFR[1] |= (AF << (4*BitNumber));
    }
}
int buttonPressed(void)
{
    if( (GPIOB->IDR & (1 << 4)) == 0) // Button is pressed
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
void SysTick_Handler(void)
{
    GPIOB->ODR |= (1 << 3);
    milliseconds++;
    GPIOB->ODR &= ~(1 << 3); // toggle PB3 for timing measurement
}
uint32_t millis()
{
    return milliseconds;
}
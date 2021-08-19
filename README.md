## stm32f4xx_drivers

Lab assignments from [Udemy](https://www.udemy.com/share/100F3uBkcacF1XR3o=/?xref=E0QfcF9RQH4HSWUuAAcqP1kSWSRM) course [MCU1](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development/) by [FastBit Embedded Brain Academy](http://fastbitlab.com/). Contains source coude for all exercises done during course, tested and adjusted for [Nucleo-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) board.


## Usage

For this assignments [Nucleo-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) board was used, but you can use code and adjust it to your board.

Src folder contains Tests, which covers every major impelmenetation (GPIO, I2C, SPI, USART, IRQ, ...)
Inside Test folder, there are Arduino sketches which are used for communication testing.

## Setup Installation - Linux Mint 19.3
* ##### GCC Toolchain installation (v7.5.0)
  * `$ gcc -v     // check if C compiler is installed or not` 
   &nbsp;
   
  * Open terminal and type following command to install GCC\
  `$ sudo apt-get install gcc`
  
* ##### STM32Cube IDE installation (v1.3.0)
  * [Download installation for Linux](https://www.st.com/en/development-tools/stm32cubeide.html) (Debian Linux Installer)
  * Navigate to downloads folder and run following command in terminal\
  `sudo chmod +x script-name-here.sh` This command will set execute permission on installation script.
  * To run installation script run one of the following commands:
    * `sudo ./script-name-here.sh`
    or
    * `sudo sh script-name-here.sh`
    or
    * `sudo bash script-name-here.sh`

* ##### *FPU* warning fix
    Right click on the project -> properties -> expand C/C++ build -> Settings -> Tool settings -> MCU settings
  * `Floating-point unit: None`
  * `Floating-point ABI: Software implementation ( -mfloat-abi=soft )`

* ##### Installing GIT (v2.7.1)
  * Install Git using apt-get:\
   `$ sudo apt-get update`\
   `$ sudo apt-get install git`
   &nbsp;
   
  * Verify the installation was successful by typing:\
  `$ git --version`
  
* ##### Installing GIT-GUI client` GitFiend`(v0.22.8)
  * Download [`GitFiend`](https://gitfiend.com/overview) *.deb* file
  * Navigate to *Downloads* folder and type `sudo apt-get install ./package-name.deb`
  * Setup `GitFiend` and connect it with your repository

&nbsp;
### *Setting up SWV ITM Data Console*

Open *syscalls.c* file and paste following code bellow *Includes*

```c
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//           Implementation of printf like feature using ARM Cortex M3/M4/ ITM functionality
//           This function will not work for ARM Cortex M0/M0+
//           If you are using Cortex M0, then you can use semihosting feature of openOCD
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//Debug Exception and Monitor Control Register base address
#define DEMCR                   *((volatile uint32_t*) 0xE000EDFCU )

/* ITM register addresses */
#define ITM_STIMULUS_PORT0   	*((volatile uint32_t*) 0xE0000000 )
#define ITM_TRACE_EN          	*((volatile uint32_t*) 0xE0000E00 )

void ITM_SendChar(uint8_t ch)
{

	//Enable TRCENA
	DEMCR |= ( 1 << 24);

	//enable stimulus port 0
	ITM_TRACE_EN |= ( 1 << 0);

	// read FIFO status in bit [0]:
	while(!(ITM_STIMULUS_PORT0 & 1));

	//Write to ITM stimulus port0
	ITM_STIMULUS_PORT0 = ch;
}
```


&nbsp;

After that find function *_write* and replace `__io_putchar(*ptr++)` with `ITM_SendChar(*ptr++)` like in code snippet below
```c
__attribute__((weak)) int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		//__io_putchar(*ptr++);
		ITM_SendChar(*ptr++);
	}
	return len;
}
```

&nbsp;

After these steps navigate to Debug configuration and check `Serial Wire Viewer (SWV)` check box like on snapshot below

![Debugger_01.png](https://github.com/nemanjadjekic/Embedded-C/blob/master/Images/Debugger_01.png)
&nbsp;

Once you enter *Debug* mode, go to `Window -> Show View -> SWV -> Select SWV ITM Data Console`. On this way `ITM Data Console` will be shown in *Debug* session.

&nbsp;

In `SWV ITM Data Console Settings` in section `ITM Stimulus Ports` enable port 0, so that you can see `printf` data.

&nbsp;

***Note: Be sure that Core Clock is properly set!***

&nbsp;
## Contributing 

Pull requests are welcome. If you discover any bug/issue feel free to report it.

## License
[MIT](https://github.com/nemanjadjekic/stm32f446xx_drivers/blob/master/LICENSE)

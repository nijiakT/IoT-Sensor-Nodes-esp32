## Assignment 2.2 
> Create a task reading the frequency from the serial monitor and storing it in the global variable. 
> Create a monitoring task: it output the current value of the number of the frequency periodically on the OLED display.

Modified from [Espressif example - uart events](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/uart/uart_events)

Reading input from the keyboard is a common demand during development. For example, setting variable values by reading from the serial port is a convenient way to try out different values instead of changing the souce code and recompiling.

The ESP32 chip comes along with three UART ports. On most dev boards, UART0 is connected to the USB, using the default gpio pin 1 and 3. The rest two UART ports could be configured to any GPIO pin. For instance, if the default pin 9,10 of UART1 are not accessible on one specific dev board, we could still use UART1 by custom pin configuration to any other available pins.

| UART NUM | Default Tx   | Default Rx   | Default Baud Rate |
| -------- | ---- | ---- | ---------- |
| uart 0   | 1    | 3    | 115200     |
| uart 1   | 10   | 9    |            |
| uart 2   | 17   | 16   |            |

#### ESP UART API functions
Those blocking functions `getchar(stdin)`, `getc()` and `scanf()` does not work on ESP32. But Espressif offers a comfort uart library for non-blocking reading purpose.
Please see [ESP UART Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html) and [UART-to-standard IO stream](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/vfs.html#standard-io-streams-stdin-stdout-stderr) for more details.

There are two possible ways to read data from the buffer:
1. read a specific length from the buffer. If you know how many bytes to be read, the `uart_read_bytes()` function could be used. 
```C
uint8_t* buf = (uint8_t*) malloc(READ_BUF_SIZE);
int buffered_size;
while(1){
    memset(buf, 0, READ_BUF_SIZE);
    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size); //How many available bytes are there in the buffer?
    uart_read_bytes(EX_UART_NUM, buf, buffered_size, portMAX_DELAY);//read into buf
    /** buffered_size could be replaced with any number, like 10 means read 10 bytes per time
    	if the wait time is set to portMAX_DELAY and not enough bytes are typed in, the function will keep waiting until enough bytes are read; otherwise it will read as many as possible before timeout.
    */
    uart_write_bytes(EX_UART_NUM, (const char*) buf, buffered_size);//serial print
}

```
2. read a line that ends with an end symbol. This is a more common use case, we want to read a full line until a specific symbol is read. Function `uart_enable_pattern_det_baud_intr()` sets the end-of-line pattern, which is a continuous string consists of the same symbol. When the pattern is deteced, some interrupt mechanism will send a `UART_PATTERN_DET` event to the uart queue.  

#### FreeRTOS Queue
The ESP UART API utilizes a queue to notify available messages in the uart buffer. A queue in FreeRTOS is a first-in-first-out buffer that transfers data between tasks. A queue features "atomic reading/writing operations" which cannot be interrupted by other tasks. This ensures thread-safety when multiple tasks have access to the same queue. In our example the queue merely serves as a way to transfer data in one direction.

More details about queue could be found at [FreeRTOS queue Documentation](https://www.freertos.org/a00018.html).

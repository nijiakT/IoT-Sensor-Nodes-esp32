#ifndef LOG_BUFFER_H

void bufferInit();

void printBuffer();
char *getBuffer();
	

int bufferAppend(const char *format, va_list args);

#define LOG_BUFFER_SIZE 1200
#define LOG_BUFFER_HELPER_SIZE 200 


#endif
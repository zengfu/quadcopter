#ifndef _LOG_H
#define _LOG_H

#include <stdio.h>

int fputc(int ch, FILE *f);
#define MPL_LOGI printf
#define MPL_LOGE printf


static inline void __print_result_location(int result,
					   const char *file,
					   const char *func, int line)
{
	MPL_LOGE("%s|%s|%d returning %d\n", file, func, line, result);
}


#define LOG_RESULT_LOCATION(condition) \
    do {								\
		__print_result_location((int)(condition), __FILE__,	\
					__func__, __LINE__);		\
	} while (0)
          
#define INV_ERROR_CHECK(r_1329) \
    if (r_1329) { \
        LOG_RESULT_LOCATION(r_1329); \
        return r_1329; \
    }



    
#endif
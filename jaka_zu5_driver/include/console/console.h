/*
 * @Author: guo.ke guo.ke@git.lotuscars.com.cn
 * @Date: 2023-05-09 16:30:24
 * @LastEditors: guo.ke guo.ke@git.lotuscars.com.cn
 * @LastEditTime: 2023-06-21 10:45:14
 * @FilePath: /rcc_2.0/src/lib/console/include/console.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#pragma once
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum 
{
    DEBUG = 0,
    INFO,
    WARN,
    ERROR,
    FATAL,
} lotus_logger_level;

// void LOTUS_DEBUG_C(const char * file, int line, char *fmt, ...);
// void LOTUS_INFO_C(const char * file,  int line, char *fmt, ...);
// void LOTUS_WARN_C(const char * file,  int line, char *fmt, ...);
// void LOTUS_ERROR_C(const char * file, int line, char *fmt, ...); 
// void LOTUS_FATAL_C(const char * file, int line, char *fmt, ...); 

void LOTUS_INFO_FORMAT(const lotus_logger_level level,  const char * file, int line, char *fmt, ...);

#define LOTUS_DEBUG_C(file, line, fmt, ...)  LOTUS_INFO_FORMAT(DEBUG,   file, line,  fmt, ##__VA_ARGS__)
#define LOTUS_INFO_C(file,  line, fmt, ...)  LOTUS_INFO_FORMAT(INFO,    file, line,  fmt, ##__VA_ARGS__)
#define LOTUS_WARN_C(file,  line, fmt, ...)  LOTUS_INFO_FORMAT(WARN,    file, line,  fmt, ##__VA_ARGS__)
#define LOTUS_ERROR_C(file, line, fmt, ...)  LOTUS_INFO_FORMAT(ERROR,   file, line,  fmt, ##__VA_ARGS__)
#define LOTUS_FATAL_C(file, line, fmt, ...)  LOTUS_INFO_FORMAT(FATAL,   file, line,  fmt, ##__VA_ARGS__)



#ifdef __cplusplus
}
#endif


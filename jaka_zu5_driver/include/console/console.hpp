/*
*
*/

#pragma once

#include <cstdio>
#include <iostream>
#include <sstream>
#include <cstdarg>
#include <map>
#include <vector>
#include <boost/make_shared.hpp>
#include <stdint.h>
#include <zlib.h>

#define LOTUSCONSOLE_DECL

#ifdef __GNUC__
#if __GNUC__ >= 3
#define LOTUSCONSOLE_PRINTF_ATTRIBUTE(a, b) __attribute__ ((__format__ (__printf__, a, b)))
#endif
#endif

#ifndef LOTUSCONSOLE_PRINTF_ATTRIBUTE
#define LOTUSCONSOLE_PRINTF_ATTRIBUTE(a, b)
#endif

namespace boost
{
    template<typename T> class shared_array;
}

namespace lotus
{
namespace console
{

namespace levels
{
enum Level
{
  Debug,
  Info,
  Warn,
  Error,
  Fatal,

  Count
};
} // namespace Level
typedef levels::Level Level;

namespace outputModules
{
enum OutputMoules
{
  M_SCREEN = 1, /* 输出到终端屏幕中 */
  M_LOG    = 2,    /* 输出到文件中 */
  M_DOUBLE = 3, /* 输出到终端屏幕和文件中 */

  M_COUNT
};
}
typedef outputModules::OutputMoules OutputMoules; /* 打印输出的模式 */

struct LogFileParam
{
  FILE * fp_type;
  std::string log_file_name;
  FILE * file_fp;
  char * file_log_buf;
  uint32_t buf_len;
  uint32_t file_cnt;
  uint32_t fflush_cnt;
  gzFile compressor;
  const bool compress_log;
  const uint32_t flush_file_size;
  const uint32_t max_file_size;
};
typedef struct LogFileParam LogFileParam;

// LOTUSCONSOLE_DECL void shutdown();

// #ifdef LOTUSCONSOLE_BACKEND_LOG4CXX
// extern LOTUSCONSOLE_DECL log4cxx::LevelPtr g_level_lookup[];
// #endif

// extern LOTUSCONSOLE_DECL bool get_loggers(std::map<std::string, levels::Level>& loggers);
// extern LOTUSCONSOLE_DECL bool set_logger_level(const std::string& name, levels::Level level);
levels::Level get_logger_level();
void set_logger_level(levels::Level level);
std::string get_logging_file(void);

/**
 * \brief Set the out put module
 */
void set_logger_output_module(outputModules::OutputMoules module);
/**
 * \brief Only exported because the maclotus need it.  Do not use directly.
 */
extern LOTUSCONSOLE_DECL bool g_initialized;

/**
 * \brief Only exported because the TopicManager need it.  Do not use directly.
 */
extern LOTUSCONSOLE_DECL std::string g_last_error_message;

class LogAppender
{
public:

  virtual ~LogAppender() {}

  virtual void log(::lotus::console::Level level, const char* str, const char* file, const char* function, int line) = 0;

};

LOTUSCONSOLE_DECL void register_appender(LogAppender* appender);

LOTUSCONSOLE_DECL void deregister_appender(LogAppender* appender);

struct Token
{
  virtual ~Token() {}
  /*
   * @param level
   * @param message
   * @param file
   * @param function
   * @param  line
   */
  virtual std::string getString(void*, ::lotus::console::Level, const char*, const char*, const char*, int) = 0;
};
typedef boost::shared_ptr<Token> TokenPtr;
typedef std::vector<TokenPtr> V_Token;

struct Formatter
{
  void init(const char* fmt);
  void print(void* logger_handle, ::lotus::console::Level level, const char* str, const char* file, const char* function, int line);
  std::string getTokenStrings(
    void *logger_handle, ::lotus::console::Level level, const char *str,
    const char *file, const char *function, int line) const;
  std::string format_;
  V_Token tokens_;
};

/**
 * \brief Only exported because the implementation need it.  Do not use directly.
 */
extern LOTUSCONSOLE_DECL Formatter g_formatter;

/**
 * \brief Don't call this directly.  Performs any required initialization/configuration.  Happens automatically when using the macro API.
 *
 * If you're going to be using log4cxx or any of the ::lotus::console functions, and need the system to be initialized, use the
 * LOTUSCONSOLE_AUTOINIT macro.
 */
LOTUSCONSOLE_DECL void initialize();

/**
 * 
 * @param process_name  进程名，用于区分不同的进程记录的日志；
 * @param format_string 日志输出的前置格式
 * @param log_file_path 日志存储的位置
 * @param output_module 日志输出的模式，终端屏幕、日志文件、终端+文件
 * @param file_max_size 单个日志文件的大小，单位MB, 当日志文件大于设定的大小时，会关闭当前日志，重新建立新的日志文件；
 * @param color_use     日志输出是否显示颜色，true 显示，false 不显示
 * 
 */

LOTUSCONSOLE_DECL void initialize(const char * process_name, 
  const char * format_string,
  const std::string & log_file_path, 
  OutputMoules output_module = OutputMoules::M_SCREEN,
  uint32_t file_max_size = 50,  
  bool color_use = true );

class FilterBase;
/**
 * \brief Don't call this directly.  Use the LOTUS_LOG() macro instead.
 * @param level Logging level
 * @param file File this logging statement is from (usually generated with __FILE__)
 * @param line Line of code this logging statement is from (usually generated with __LINE__)
 * @param fmt Format string
 */
LOTUSCONSOLE_DECL void print(FilterBase* filter, void* logger, Level level,
	   const char* file, int line, 
	   const char* function, const char* fmt, ...) LOTUSCONSOLE_PRINTF_ATTRIBUTE(7, 8);

LOTUSCONSOLE_DECL void print(FilterBase* filter, void* logger, Level level,
	   const std::stringstream& str, const char* file, int line, const char* function);

struct LOTUSCONSOLE_DECL LogLocation;

/**
 * \brief Registers a logging location with the system.
 *
 * This is used for the case where a logger's verbosity level changes, and we need to reset the enabled status of
 * all the logging statements.
 * @param loc The location to add
 */
LOTUSCONSOLE_DECL void registerLogLocation(LogLocation* loc);

/**
 * \brief Tells the system that a logger's level has changed
 *
 * This must be called if a log4cxx::Logger's level has been changed in the middle of an application run.
 * Because of the way the static guard for enablement works, if a logger's level is changed and this
 * function is not called, only logging statements which are first hit *after* the change will be correct wrt
 * that logger.
 */
LOTUSCONSOLE_DECL void notifyLoggerLevelsChanged();

LOTUSCONSOLE_DECL void setFixedFilterToken(const std::string& key, const std::string& val);

/**
 * \brief Parameter structure passed to FilterBase::isEnabled(...);.  Includes both input and output parameters
 */
struct FilterParams
{
  // input parameters
  const char* file;                         ///< [input] File the message came from
  int line;                                 ///< [input] Line the message came from
  const char* function;                     ///< [input] Function the message came from
  const char* message;                      ///< [input] The formatted message that will be output

  // input/output parameters
  void* logger;                             ///< [input/output] Handle identifying logger that this message will be output to.  If changed, uses the new logger
  Level level;                              ///< [input/output] Severity level.  If changed, uses the new level

  // output parameters
  std::string out_message;                  ///< [output] If set, writes this message instead of the original
};

/**
 * \brief Base-class for filters.  Filters allow full user-defined control over whether or not a message should print.
 * The LOTUS_X_FILTER... maclotus provide the filtering functionality.
 *
 * Filters get a chance to veto the message from printing at two times: first before the message arguments are
 * evaluated and the message is formatted, and then once the message is formatted before it is printed.  It is also possible
 * to change the message, logger and severity level at this stage (see the FilterParams struct for more details).
 *
 * When a LOTUS_X_FILTER... macro is called, here is the high-level view of how it uses the filter passed in:
\verbatim
if (<logging level is enabled> && filter->isEnabled())
{
  <format message>
  <fill out FilterParams>
  if (filter->isEnabled(params))
  {
    <print message>
  }
}
\endverbatim
 */
class FilterBase
{
public:
  virtual ~FilterBase() {}
  /**
   * \brief Returns whether or not the log statement should be printed.  Called before the log arguments are evaluated
   * and the message is formatted.
   */
  inline virtual bool isEnabled() { return true; }
  /**
   * \brief Returns whether or not the log statement should be printed.  Called once the message has been formatted,
   * and allows you to change the message, logger and severity level if necessary.
   */
  inline virtual bool isEnabled(FilterParams&) { return true; }
};

struct LOTUSCONSOLE_DECL LogLocation;
/**
 * \brief Internal
 */
LOTUSCONSOLE_DECL void initializeLogLocation(LogLocation* loc, const std::string& name, Level level);
/**
 * \brief Internal
 */
LOTUSCONSOLE_DECL void setLogLocationLevel(LogLocation* loc, Level level);
/**
 * \brief Internal
 */
LOTUSCONSOLE_DECL void checkLogLocationEnabled(LogLocation* loc);

/**
 * \brief Internal
 */
struct LogLocation
{
  bool initialized_;
  bool logger_enabled_;
  ::lotus::console::Level level_;
  void* logger_;
};

LOTUSCONSOLE_DECL void vformatToBuffer(boost::shared_array<char>& buffer, size_t& buffer_size, const char* fmt, va_list args);
LOTUSCONSOLE_DECL void formatToBuffer(boost::shared_array<char>& buffer, size_t& buffer_size, const char* fmt, ...);
LOTUSCONSOLE_DECL std::string formatToString(const char* fmt, ...);

} // namespace console
} // namespace lotus

#define LOTUS_LIKELY(x)       __builtin_expect((x),1)
#define LOTUS_UNLIKELY(x)     __builtin_expect((x),0)

#if defined(_MSC_VER)
  #define __LOTUSCONSOLE_FUNCTION__ __FUNCSIG__
#elif defined(__GNUC__)
  #define __LOTUSCONSOLE_FUNCTION__ __PRETTY_FUNCTION__
#else
  #define __LOTUSCONSOLE_FUNCTION__ ""
#endif

#ifdef LOTUS_PACKAGE_NAME
#define LOTUSCONSOLE_PACKAGE_NAME LOTUS_PACKAGE_NAME
#else
#define LOTUSCONSOLE_PACKAGE_NAME "unknown_package"
#endif

#define LOTUSCONSOLE_ROOT_LOGGER_NAME "lotus"
#define LOTUSCONSOLE_NAME_PREFIX LOTUSCONSOLE_ROOT_LOGGER_NAME "." LOTUSCONSOLE_PACKAGE_NAME
#define LOTUSCONSOLE_DEFAULT_NAME LOTUSCONSOLE_NAME_PREFIX

// These allow you to compile-out everything below a certain severity level if necessary
#define LOTUSCONSOLE_SEVERITY_DEBUG 0
#define LOTUSCONSOLE_SEVERITY_INFO 1
#define LOTUSCONSOLE_SEVERITY_WARN 2
#define LOTUSCONSOLE_SEVERITY_ERROR 3
#define LOTUSCONSOLE_SEVERITY_FATAL 4
#define LOTUSCONSOLE_SEVERITY_NONE 5

/**
 * \def LOTUSCONSOLE_MIN_SEVERITY
 *
 * Define LOTUSCONSOLE_MIN_SEVERITY=LOTUSCONSOLE_SEVERITY_[DEBUG|INFO|WARN|ERROR|FATAL] in your build options to compile out anything below that severity
 */
#ifndef LOTUSCONSOLE_MIN_SEVERITY
#define LOTUSCONSOLE_MIN_SEVERITY LOTUSCONSOLE_SEVERITY_DEBUG
#endif

/**
 * \def LOTUSCONSOLE_AUTOINIT
 * \brief Initializes the lotusconsole library.  Usually unnecessary to call directly.
 */
#define LOTUSCONSOLE_AUTOINIT \
  do \
  { \
    if (LOTUS_UNLIKELY(!::lotus::console::g_initialized)) \
    { \
      ::lotus::console::initialize(); \
    } \
  } while(false)

#define LOTUSCONSOLE_DEFINE_LOCATION(cond, level, name) \
  LOTUSCONSOLE_AUTOINIT; \
  static ::lotus::console::LogLocation __lotusconsole_define_location__loc = {false, false, ::lotus::console::levels::Count, NULL}; /* Initialized at compile-time */ \
  if (LOTUS_UNLIKELY(!__lotusconsole_define_location__loc.initialized_)) \
  { \
    initializeLogLocation(&__lotusconsole_define_location__loc, name, level); \
  } \
  if (LOTUS_UNLIKELY(__lotusconsole_define_location__loc.level_ != level)) \
  { \
    setLogLocationLevel(&__lotusconsole_define_location__loc, level); \
    checkLogLocationEnabled(&__lotusconsole_define_location__loc); \
  } \
  bool __lotusconsole_define_location__enabled = __lotusconsole_define_location__loc.logger_enabled_ && (cond);

#define LOTUSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER(filter, ...) \
    ::lotus::console::print(filter, __lotusconsole_define_location__loc.logger_, __lotusconsole_define_location__loc.level_, __FILE__, __LINE__, __LOTUSCONSOLE_FUNCTION__, __VA_ARGS__)

#define LOTUSCONSOLE_PRINT_AT_LOCATION(...) \
    LOTUSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER(NULL, __VA_ARGS__)

// inside a macro which uses args use only well namespaced variable names in order to not overlay variables coming in via args
#define LOTUSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(filter, args) \
  do \
  { \
    std::stringstream __lotusconsole_print_stream_at_location_with_filter__ss__; \
    __lotusconsole_print_stream_at_location_with_filter__ss__ << args; \
    ::lotus::console::print(filter, __lotusconsole_define_location__loc.logger_, __lotusconsole_define_location__loc.level_, __lotusconsole_print_stream_at_location_with_filter__ss__, __FILE__, __LINE__, __LOTUSCONSOLE_FUNCTION__); \
  } while (0)

#define LOTUSCONSOLE_PRINT_STREAM_AT_LOCATION(args) \
    LOTUSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(NULL, args)

/**
 * \brief Used internally in throttle maclotus to determine if a new message should be logged.
 * \note If LOTUS time has moved backwards, will evaluate to true
 */
// #define LOTUSCONSOLE_THROTTLE_CHECK(now, last, period) (LOTUS_UNLIKELY(last + period <= now) || LOTUS_UNLIKELY(now < last))

/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with printf-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define LOTUS_LOG_COND(cond, level, name, ...) \
  do \
  { \
    LOTUSCONSOLE_DEFINE_LOCATION(cond, level, name); \
    \
   if (LOTUS_UNLIKELY(__lotusconsole_define_location__enabled)) \
    { \
      LOTUSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(false)

/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with stream-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define LOTUS_LOG_STREAM_COND(cond, level, name, args) \
  do \
  { \
    LOTUSCONSOLE_DEFINE_LOCATION(cond, level, name); \
    if (LOTUS_UNLIKELY(__lotusconsole_define_location__enabled)) \
    { \
      LOTUSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
    } \
  } while(false)

/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with printf-style formatting
 *
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */

#define LOTUS_LOG_ONCE(level, name, ...) \
  do \
  { \
    LOTUSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static bool hit = false; \
    if (LOTUS_UNLIKELY(__lotusconsole_define_location__enabled) && LOTUS_UNLIKELY(!hit)) \
    { \
      hit = true; \
      LOTUSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(false)
 

// inside a macro which uses args use only well namespaced variable names in order to not overlay variables coming in via args
/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with printf-style formatting
 *
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define LOTUS_LOG_STREAM_ONCE(level, name, args) \
  do \
  { \
    LOTUSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static bool __lotus_log_stream_once__hit__ = false; \
    if (LOTUS_UNLIKELY(__lotusconsole_define_location__enabled) && LOTUS_UNLIKELY(!__lotus_log_stream_once__hit__)) \
    { \
      __lotus_log_stream_once__hit__ = true; \
      LOTUSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
    } \
  } while(false)


/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param period The period it should actually trigger at most. If LOTUS time has moved backwards, it will trigger regardless.
 */

/*
// #define LOTUS_LOG_THROTTLE(period, level, name, ...) \
//   do \
//   { \
//     LOTUSCONSOLE_DEFINE_LOCATION(true, level, name); \
//     static double __lotus_log_throttle_last_hit__ = 0.0; \
//     double __lotus_log_throttle_now__ = ::lotus::Time::now().toSec(); \
//     if (LOTUS_UNLIKELY(__lotusconsole_define_location__enabled) && LOTUSCONSOLE_THROTTLE_CHECK(__lotus_log_throttle_now__, __lotus_log_throttle_last_hit__, period))\
//     { \
//       __lotus_log_throttle_last_hit__ = __lotus_log_throttle_now__; \
//       LOTUSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
//     } \
//   } while(false)
*/

// inside a macro which uses args use only well namespaced variable names in order to not overlay variables coming in via args
/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param period The period it should actually trigger at most. If LOTUS time has moved backwards, it will trigger regardless.
 */
/*
// #define LOTUS_LOG_STREAM_THROTTLE(period, level, name, args) \
//   do \
//   { \
//     LOTUSCONSOLE_DEFINE_LOCATION(true, level, name); \
//     static double __lotus_log_stream_throttle__last_hit__ = 0.0; \
//     double __lotus_log_stream_throttle__now__ = ::lotus::Time::now().toSec(); \
//     if (LOTUS_UNLIKELY(__lotusconsole_define_location__enabled) && \
//         LOTUSCONSOLE_THROTTLE_CHECK(__lotus_log_stream_throttle__now__, __lotus_log_stream_throttle__last_hit__, period))\
//     { \
//       __lotus_log_stream_throttle__last_hit__ = __lotus_log_stream_throttle__now__; \
//       LOTUSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
//     } \
//   } while(false)
*/

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param period The period it should actually trigger at most, and the delay before which no message will be shown. If LOTUS time has moved backwards, it will trigger regardless.
 */
/**
// #define LOTUS_LOG_DELAYED_THROTTLE(period, level, name, ...) \
//   do \
//   { \
//     LOTUSCONSOLE_DEFINE_LOCATION(true, level, name); \
//     double __lotus_log_delayed_throttle__now__ = ::lotus::Time::now().toSec(); \
//     static double __lotus_log_delayed_throttle__last_hit__ = __lotus_log_delayed_throttle__now__; \
//     if (LOTUS_UNLIKELY(__lotusconsole_define_location__enabled) && \
//         LOTUSCONSOLE_THROTTLE_CHECK(__lotus_log_delayed_throttle__now__, __lotus_log_delayed_throttle__last_hit__, period))\
//     { \
//       __lotus_log_delayed_throttle__last_hit__ = __lotus_log_delayed_throttle__now__; \
//       LOTUSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
//     } \
//   } while(false)
*/

// inside a macro which uses args use only well namespaced variable names in order to not overlay variables coming in via args
/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing and postponed first message
 *
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param period The period it should actually trigger at most, and the delay before which no message will be shown. If LOTUS time has moved backwards, it will trigger regardless.
 */
/*
// #define LOTUS_LOG_STREAM_DELAYED_THROTTLE(period, level, name, args) \
//   do \
//   { \
//     LOTUSCONSOLE_DEFINE_LOCATION(true, level, name); \
//     double __lotus_log_stream_delayed_throttle__now__ = ::lotus::Time::now().toSec(); \
//     static double __lotus_log_stream_delayed_throttle__last_hit__ = __lotus_log_stream_delayed_throttle__now__; \
//     if (LOTUS_UNLIKELY(__lotusconsole_define_location__enabled) && \
//         LOTUSCONSOLE_THROTTLE_CHECK(__lotus_log_stream_delayed_throttle__now__, __lotus_log_stream_delayed_throttle__last_hit__, period)) \
//     { \
//       __lotus_log_stream_delayed_throttle__last_hit__ = __lotus_log_stream_delayed_throttle__now__; \
//       LOTUSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
//     } \
//   } while(false)
*/
/**
 * \brief Log to a given named logger at a given verbosity level, with user-defined filtering, with printf-style formatting
 *
 * \param filter pointer to the filter to be used
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
/*
// #define LOTUS_LOG_FILTER(filter, level, name, ...) \
//   do \
//   { \
//     LOTUSCONSOLE_DEFINE_LOCATION(true, level, name); \
//     if (LOTUS_UNLIKELY(__lotusconsole_define_location__enabled) && (filter)->isEnabled()) \
//     { \
//       LOTUSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER(filter, __VA_ARGS__); \
//     } \
//   } while(false)
*/
/**
 * \brief Log to a given named logger at a given verbosity level, with user-defined filtering, with stream-style formatting
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
/*
// #define LOTUS_LOG_STREAM_FILTER(filter, level, name, args) \
//   do \
//   { \
//     LOTUSCONSOLE_DEFINE_LOCATION(true, level, name); \
//     if (LOTUS_UNLIKELY(__lotusconsole_define_location__enabled) && (filter)->isEnabled()) \
//     { \
//       LOTUSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(filter, args); \
//     } \
//   } while(false)
*/

/**
 * \brief Log to a given named logger at a given verbosity level, with printf-style formatting
 *
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define LOTUS_LOG(level, name, ...) LOTUS_LOG_COND(true, level, name, __VA_ARGS__)
/**
 * \brief Log to a given named logger at a given verbosity level, with stream-style formatting
 *
 * \param level One of the levels specified in ::lotus::console::levels::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "lotus.<package_name>".  Use LOTUSCONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define LOTUS_LOG_STREAM(level, name, args) LOTUS_LOG_STREAM_COND(true, level, name, args)


#define LOTUS_DEBUG(...) LOTUS_LOG(::lotus::console::levels::Debug, LOTUSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOTUS_INFO(...)  LOTUS_LOG(::lotus::console::levels::Info,  LOTUSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOTUS_WARN(...)  LOTUS_LOG(::lotus::console::levels::Warn,  LOTUSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOTUS_ERROR(...) LOTUS_LOG(::lotus::console::levels::Error, LOTUSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOTUS_FATAL(...) LOTUS_LOG(::lotus::console::levels::Fatal, LOTUSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

#define LOTUS_DEBUG_ONCE(...) LOTUS_LOG_ONCE(::lotus::console::levels::Debug, LOTUSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOTUS_INFO_ONCE(...)  LOTUS_LOG_ONCE(::lotus::console::levels::Info,  LOTUSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOTUS_WARN_ONCE(...)  LOTUS_LOG_ONCE(::lotus::console::levels::Warn,  LOTUSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOTUS_ERROR_ONCE(...) LOTUS_LOG_ONCE(::lotus::console::levels::Error, LOTUSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define LOTUS_FATAL_ONCE(...) LOTUS_LOG_ONCE(::lotus::console::levels::Fatal, LOTUSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

#define LOTUS_DEBUG_STREAM(args)  LOTUS_LOG_STREAM(::lotus::console::levels::Debug, LOTUSCONSOLE_DEFAULT_NAME, args)
#define LOTUS_INFO_STREAM(args)   LOTUS_LOG_STREAM(::lotus::console::levels::Info,  LOTUSCONSOLE_DEFAULT_NAME, args)
#define LOTUS_WARN_STREAM(args)   LOTUS_LOG_STREAM(::lotus::console::levels::Warn,  LOTUSCONSOLE_DEFAULT_NAME, args)
#define LOTUS_ERROR_STREAM(args)  LOTUS_LOG_STREAM(::lotus::console::levels::Error, LOTUSCONSOLE_DEFAULT_NAME, args)
#define LOTUS_FATAL_STREAM(args)  LOTUS_LOG_STREAM(::lotus::console::levels::Fatal, LOTUSCONSOLE_DEFAULT_NAME, args)

#define LOTUS_DEBUG_STREAM_ONCE(args)  LOTUS_LOG_STREAM_ONCE(::lotus::console::levels::Debug, LOTUSCONSOLE_DEFAULT_NAME, args)
#define LOTUS_INFO_STREAM_ONCE(args)   LOTUS_LOG_STREAM_ONCE(::lotus::console::levels::Info,  LOTUSCONSOLE_DEFAULT_NAME, args)
#define LOTUS_WARN_STREAM_ONCE(args)   LOTUS_LOG_STREAM_ONCE(::lotus::console::levels::Warn,  LOTUSCONSOLE_DEFAULT_NAME, args)
#define LOTUS_ERROR_STREAM_ONCE(args)  LOTUS_LOG_STREAM_ONCE(::lotus::console::levels::Error, LOTUSCONSOLE_DEFAULT_NAME, args)
#define LOTUS_FATAL_STREAM_ONCE(args)  LOTUS_LOG_STREAM_ONCE(::lotus::console::levels::Fatal, LOTUSCONSOLE_DEFAULT_NAME, args)

// #include "lotusconsole/maclotus_generated.h"
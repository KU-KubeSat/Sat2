/* WARNING! All changes made to this file will be lost! */

#ifndef W_INCLUDE_CSP_CSP_AUTOCONFIG_H_WAF
#define W_INCLUDE_CSP_CSP_AUTOCONFIG_H_WAF

#define GIT_REV "f1c07df"

#if  defined(__linux__)
  #define CSP_POSIX 1
#elif defined(__WIN32__)
  #define CSP_WINDOWS 1
#elif defined(__APPLE__)
  #define CSP_MACOSX 1
#else
  #define CSP_FREERTOS 1
#endif
#define CSP_DEBUG 1
#define CSP_DEBUG_TIMESTAMP 0
#define CSP_USE_RDP 1
#define CSP_USE_RDP_FAST_CLOSE 1
#define CSP_USE_CRC32 1
#define CSP_USE_HMAC 1
#define CSP_USE_XTEA 1
#define CSP_USE_PROMISC 0
#define CSP_USE_QOS 0
#define CSP_USE_DEDUP 0
#define CSP_USE_EXTERNAL_DEBUG 0
#define CSP_LOG_LEVEL_DEBUG 1
#define CSP_LOG_LEVEL_INFO 1
#define CSP_LOG_LEVEL_WARN 1
#define CSP_LOG_LEVEL_ERROR 1
#define CSP_LITTLE_ENDIAN 1
/* #undef CSP_BIG_ENDIAN */
#define LIBCSP_VERSION "1.6"

#endif /* W_INCLUDE_CSP_CSP_AUTOCONFIG_H_WAF */

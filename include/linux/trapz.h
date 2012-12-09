/*
 * trapz.h
 *
 * TRAPZ (TRAcing and Profiling for Zpeed) Log Driver header file
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Andy Prunicki (prunicki@lab126.com)
 * Martin Unsal (munsal@lab126.com)
 * TODO: Add additional contributor's names.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * This file must be #included anywhere that TRAPZ tracing calls are made from C
 * or C++ code.  Currently that includes the Linux kernel build and the Android
 * build.
 *
 * You must also make sure that your source code is scanned by the TRAPZ tool
 * chain. This is accomplished by editing labscripts/trapz/trapz.xml and adding
 * a <component> tag for your project with a <scan> tag pointing at your source
 * code.
 */

#ifndef _LINUX_TRAPZ_H
#define _LINUX_TRAPZ_H

/**
 * CONFIG_TRAPZ is used to enable the compilation of the TRAPZ kernel driver.
 * ENABLE_TRAPZ is used to turn TRAPZ logging on and off.
 */

#include <linux/types.h>
#include <linux/time.h>
#include <linux/ioctl.h>
#ifndef __KERNEL__
#include <sys/syscall.h>
#endif

#ifdef ENABLE_TRAPZ
#ifdef __KERNEL__
#include <linux/trapz_generated_kernel.h>
#else
#include <linux/trapz_generated.h>
#endif /* __KERNEL__ */
#endif /* ENABLE_TRAPZ */

#define TRAPZ_MAX_COMP_ID         0xfff         // 4095
#define TRAPZ_DEFAULT_BUFFER_SIZE 10000         // ~200kb
#define TRAPZ_PROC_INFO           "trapzinfo"
#define TRAPZ_PROC_DATA           "trapzdata"
#define TRAPZ_PROC_TEXT           "trapztext"
#define TRAPZ_DEV_NAME            "trapz"

typedef struct trapz_info {
	struct timespec tv;
	int sequence;
} trapz_info_t;

typedef struct {
	struct timespec tv;         // time stamp
	int sequence;               // assigned sequentially
	int ctrl1;                  // ctrl1
	int ctrl2;                  // ctrl2
	int extra1;                 // entry specific extra info
	int extra2;
	int pid;
	int tid;
	unsigned int cpu;
} trapz_entry_t;

typedef struct {
	trapz_entry_t entry;        // entry
	int overtaken;              // count of entries overtaken due to buffer wrap
	int remaining;              // count of remaining entries
	int total;                  // this many entries have been added since the last reset
} trapz_read_entry_t;

typedef struct {
	int start_trace_point;
	int end_trace_point;
	int trigger_id;
	int single_shot;
} trapz_trigger_t;

typedef struct {
	trapz_trigger_t trigger;
	struct timespec start_ts;
	struct timespec end_ts;
	int trigger_active;
} trapz_trigger_event_t;

#ifdef __KERNEL__
/* Internal kernel API to register events */
long systrapz(int ctrl1, int ctrl2, int extra1, int extra2, trapz_info_t __user *ti);
/* Checks if a trapz component is enabled for a given loglevel */
int trapz_check_loglevel(const int level, const int cat_id, const int component_id);
/* actual system call interface (see kernel/sys_ni.c) */
long sys_trapz(int ctrl1, int ctrl2, int extra1, int extra2, int reserved, struct trapz_info __user *ti);
#else /* __KERNEL */
#  define __NR_trapz 98
inline int systrapz(int ctrl1, int ctrl2, int extra1, int extra2, trapz_info_t *ti)
{
  return syscall(__NR_trapz, ctrl1, ctrl2, extra1, extra2, NULL, ti);
}
#endif

#define TRAPZ_LEVEL_SIZE        2
#define TRAPZ_LFLAGS_SIZE       3
#define TRAPZ_CAT_ID_SIZE       2
#define TRAPZ_COMP_ID_SIZE     12
#define TRAPZ_TRACE_ID_SIZE    12

/* ctrl1 log field input masks */
#define TRAPZ_LLEVEL_IN_MASK      0x00000003
#define TRAPZ_LFLAGS_IN_MASK      0x00000007
#define TRAPZ_CAT_ID_IN_MASK      0x00000003
#define TRAPZ_COMP_ID_IN_MASK     0x00000FFF
#define TRAPZ_TRACE_ID_IN_MASK    0x00000FFF

/* ctrl2 log field input masks */

/* ctrl1 log field output masks */
#define TRAPZ_LLEVEL_OUT_MASK     0x60000000
#define TRAPZ_LFLAGS_OUT_MASK     0x1c000000
#define TRAPZ_CAT_ID_OUT_MASK     0x03000000
#define TRAPZ_COMP_ID_OUT_MASK    0x00fff000
#define TRAPZ_TRACE_ID_OUT_MASK   0x00000fff

/* ctrl1 log field offsets */
#define TRAPZ_LLEVEL_OFFSET     29
#define TRAPZ_LFLAGS_OFFSET     26
#define TRAPZ_CAT_ID_OFFSET     24
#define TRAPZ_COMP_ID_OFFSET    12
#define TRAPZ_TRACE_ID_OFFSET    0

/* ctrl2 log field offsets */

/* ctrl1 log field getters */
#define TRAPZ_GET_LLEVEL(x) ((x & TRAPZ_LLEVEL_OUT_MASK) >> TRAPZ_LLEVEL_OFFSET)
#define TRAPZ_GET_LFLAGS(x) ((x & TRAPZ_LFLAGS_OUT_MASK) >> TRAPZ_LFLAGS_OFFSET)
#define TRAPZ_GET_CAT_ID(x) ((x & TRAPZ_CAT_ID_OUT_MASK) >> TRAPZ_CAT_ID_OFFSET)
#define TRAPZ_GET_COMP_ID(x) ((x & TRAPZ_COMP_ID_OUT_MASK) >> TRAPZ_COMP_ID_OFFSET)
#define TRAPZ_GET_TRACE_ID(x) ((x & TRAPZ_TRACE_ID_OUT_MASK) >> TRAPZ_TRACE_ID_OFFSET)

/* ctrl2 log field getters */

/* ctrl1 log field setters */
#define TRAPZ_LLEVEL_IN(x) ((x & TRAPZ_LLEVEL_IN_MASK) << TRAPZ_LLEVEL_OFFSET)
#define TRAPZ_LFLAGS_IN(x) ((x & TRAPZ_LFLAGS_IN_MASK) << TRAPZ_LFLAGS_OFFSET)
#define TRAPZ_CAT_ID_IN(x) ((x & TRAPZ_CAT_ID_IN_MASK) << TRAPZ_CAT_ID_OFFSET)
#define TRAPZ_COMP_ID_IN(x) ((x & TRAPZ_COMP_ID_IN_MASK) << TRAPZ_COMP_ID_OFFSET)
#define TRAPZ_TRACE_ID_IN(x) ((x & TRAPZ_TRACE_ID_IN_MASK) << TRAPZ_TRACE_ID_OFFSET)

/* ctrl2 log field setters */

#define TRAPZ_TRIGGER_MASK (TRAPZ_CAT_ID_OUT_MASK | TRAPZ_COMP_ID_OUT_MASK | TRAPZ_TRACE_ID_OUT_MASK)

/* Log Macro definitions
 */
/* A simplified macro which uses the component category defined in trapz_generated.h */
#define TRAPZ_ILOG(level, flags, component, trace_id, strace_id, extra1, extra2) \
  trapz_ilog(level, flags, component##__CAT, component##__ID, trace_id, strace_id, extra1, extra2)

/* The following macros depend on a pre-compilation step to generate
 * numeric trace IDs from human-readable identifiers.
 *
 * Each macro takes an argument "trace" which is an identifier that
 * describes the purpose of the log. This identifier doesn't need to
 * be valid in the current scope.  For example:
 *
 * TRAPZ_LOG(TRAPZ_LOG_INFO, 0, TRAPZ_KERN, descriptive_phrase_here, 0, 0)
 *
 * After adding any new log statement, you must run 'make trapz' at
 * top level before building your code. The toolchain will parse your
 * TRAPZ_LOG call and #define the symbol you have used to an
 * autogenerated unique ID. In this case it would generate something
 * like this:
 *
 * #define TRAPZ_KERN___descriptive_phrase_here 3517
 *
 * XXXmunsal Be aware the parser is for the moment extremely crude and
 * not aware of C syntax. For example, if you have a TRAPZ_LOG
 * call that has been commented out, trapztool will still try to parse
 * it. Also the parser will be confused by extra commas contained
 * within nested expressions, such as here:
 *
 * TRAPZ_LOG(calculateLogLevel(foo,bar,baz), 0, TRAPZ_KERN, whatever, 0, 0)
 *
 * As far as my shitty parser is concerned, the first argument to
 * TRAPZ_LOG is 'calculateLogLevel(foo'. Epic fail.
 *
 * XXXmunsal Be aware that 'trace' must currently be a valid identifier string
 * in both C and Verilog (it ends up going in VCD file). I need to investigate
 * whether there are valid C identifiers that aren't valid Verilog identifiers
 * and either document that or convert them in toolchain.
 */
#define TRAPZ_LOG(level, flags, component, trace, extra1, extra2) \
  TRAPZ_ILOG(level, flags, component, component##___##trace, 0, extra1, extra2)

/* Macro for logging with a printf string. The string is ignored in
 * compilation. It is used by code analysis tools to make the TRAPZ
 * log more readable.
 *
 * TRAPZ_LOG_PRINTF(TRAPZ_LOG_INFO, 0, TRAPZ_KERN_TOUCH, touch,
 *                  "Touch event in function Foo at %d %d", x, y)
 *
 * XXXmunsal I will warn again... due to my shitty parser you MUST
 * NOT have any commas or parentheses in your format string!
 *
 * XXXmunsal Be aware that format string will actually be formatted
 * by Python % operator, not C printf(). They are broadly compatible
 * but you're best off keeping it simple. I need to investigate
 * differences and document them here.
 */
#define TRAPZ_LOG_PRINTF(level, flags, component, trace, format, arg1, arg2) \
  TRAPZ_ILOG(level, flags, component, component##___##trace, 0, arg1, arg2)

/* Macros for logging an interval. Similar to TRAPZ_LOG but will
 * be handled intelligently by postprocessing tools.
 */
#define TRAPZ_LOG_BEGIN(level, flags, component, trace)	\
  TRAPZ_ILOG(level, flags, component, component##___##trace, 0, 0, 1)
#define TRAPZ_LOG_END(level, flags, component, trace) \
  TRAPZ_ILOG(level, flags, component, component##___##trace, 0, 0, 0)

/* Macros for logging function scope. These are slightly different, in
 * that the identifier used to generate trace IDs MUST be a valid C
 * identifier; it should be the name of the enclosing function. This
 * is used to embed the function pointer in the log. Example:
 *
 * void myFunction(void) {
 *   TRAPZ_LOG_ENTER(TRAPZ_LOG_INFO, 0, TRAPZ_KERN, myFunction);
 *   do_stuff();
 *   if (stuff_failed()) {
 *     TRAPZ_LOG_FAIL(TRAPZ_LOG_INFO, 0, TRAPZ_KERN, myFunction);
 *     return;
 *   }
 *   TRAPZ_LOG_EXIT(TRAPZ_LOG_INFO, 0, TRAPZ_KERN, myFunction);
 * }
 *
 * XXXmunsal The utility of these macros is unclear. Unless we have
 * postprocessing tools that use symbol tables, the function pointr is pretty
 * useless to us. I expect these will eventually move into TRAPZ_LOG_BEGIN/END.
 */
#define TRAPZ_LOG_ENTER(level, flags, component, fn_ptr) \
  TRAPZ_ILOG(level, flags, component, component##___##fn_ptr, 0, (int)fn_ptr, 1)
#define TRAPZ_LOG_EXIT(level, flags, component, fn_ptr) \
  TRAPZ_ILOG(level, flags, component, component##___##fn_ptr, 0, (int)fn_ptr, 0)
#define TRAPZ_LOG_FAIL(level, flags, component, fn_ptr) \
  TRAPZ_ILOG(level, flags, component, component##___##fn_ptr, 0, (int)fn_ptr, -1)

/* Macro for generating documentation of a Trapz trace
 *
 * E.g.:
 *
 * TRAPZ_DESCRIBE(TRAPZ_KERN, my_log,
 * "Here is a multi line string literal talking about"
 * "just what this log point means."
 * "You can even use <strong>HTML TAGS!</strong>")
 * TRAPZ_LOG(TRAPZ_LOG_INFO, 0, TRAPZ_KERN, my_log, 0, 0)
 */
#define TRAPZ_DESCRIBE(component, trace_or_fn_ptr, description)

/* Lowest level macros. This is where ENABLE_TRAPZ takes effect. */
#ifdef ENABLE_TRAPZ

#define trapz_ilog(level, flags, cat_id, comp_id, trace_id, strace_id, extra1, extra2)  \
  trapz_ilog_info(level, flags, cat_id, comp_id, trace_id, strace_id, extra1, extra2, NULL)

#define trapz_ilog_info(level, flags, cat_id, comp_id, trace_id, strace_id, extra1, extra2, trapzinfo)  \
  systrapz(                             \
        TRAPZ_LLEVEL_IN(level) |        \
        TRAPZ_LFLAGS_IN(flags) |        \
        TRAPZ_CAT_ID_IN(cat_id) |       \
        TRAPZ_COMP_ID_IN(comp_id) |     \
        TRAPZ_TRACE_ID_IN(trace_id),    \
        0, extra1, extra2, trapzinfo)

#else /* ENABLE_TRAPZ */

#define trapz_ilog(level, flags, cat_id, comp_id, trace_id, strace_id, extra1, extra2)
#define trapz_ilog_info(level, flags, cat_id, comp_id, trace_id, strace_id, extra1, extra2, trapzinfo)

#endif /* ENABLE_TRAPZ */

/* Category definitions
 */
#define TRAPZ_CAT_KERNEL         0x0
#define TRAPZ_CAT_PLATFORM       0x1
#define TRAPZ_CAT_APPS           0x2

/* Flags definitions
 */
#define TRAPZ_FLAG_LOG_AL         0x1   // Copy to main Android log
#define TRAPZ_FLAG_LOG_KL         0x2   // Copy to kernel log
#define TRAPZ_FLAG_LOG_KT         0x4   // Copy to kernel trace

#ifndef __KERNEL__

#define TRAPZ_FLAG_LOG_AL_TEXT         "Log-Android"
#define TRAPZ_FLAG_LOG_KL_TEXT         "Log-Kernel"
#define TRAPZ_FLAG_LOG_KT_TEXT         "Log-Trace"

static struct trapz_flag_text_map_entry_struct {
  int flag;
  const char *txt;
} trapz_flag_text_map[] = {
  {TRAPZ_FLAG_LOG_AL, TRAPZ_FLAG_LOG_AL_TEXT},
  {TRAPZ_FLAG_LOG_KL, TRAPZ_FLAG_LOG_KL_TEXT},
  {TRAPZ_FLAG_LOG_KT, TRAPZ_FLAG_LOG_KT_TEXT},
};
#endif /* ! __KERNEL__ */

/* Level definitions
 */
#define TRAPZ_LOG_MAX		3
#define TRAPZ_LOG_MIN		0
#define TRAPZ_LOG_WARN		3
#define TRAPZ_LOG_INFO		2
#define TRAPZ_LOG_DEBUG		1
#define TRAPZ_LOG_VERBOSE	0

typedef struct {
	int bufferSize;         // the buffer will hold this many entries
	int count;              // the buffer contains this many entries
	int total;              // count of entries added since the last reset
	int overtaken;          // count of entries overtaken due to buffer wrap
} trapz_config;

#define __TRAPZIO	0xAF

/* IOCTL codes */
#define TRAPZ_GET_CONFIG         _IO(__TRAPZIO, 1)  /* size of log */
#define TRAPZ_SET_BUFFER_SIZE    _IO(__TRAPZIO, 2)  /* used log len */
#define TRAPZ_RESET_DEFAULTS     _IO(__TRAPZIO, 3)  /* reset to defaults - log level, filters, and triggers */
#define TRAPZ_CLEAR_BUFFER       _IO(__TRAPZIO, 4)  /* clear buffer */
#define TRAPZ_GET_VERSION        _IO(__TRAPZIO, 5)  /* get TRAPZ version */
#define TRAPZ_SET_LOG_LEVEL      _IO(__TRAPZIO, 6)  /* set minimum log level */
#define TRAPZ_CHK_LOG_LEVEL      _IO(__TRAPZIO, 7)  /* check log level */
#define TRAPZ_ADD_TRIGGER        _IO(__TRAPZIO, 9)  /* add trigger */
#define TRAPZ_DEL_TRIGGER        _IO(__TRAPZIO, 10) /* delete trigger */
#define TRAPZ_CLR_TRIGGERS       _IO(__TRAPZIO, 11) /* clear all triggers */
#define TRAPZ_CNT_TRIGGERS       _IO(__TRAPZIO, 12) /* count triggers */

#endif  /* _LINUX_TRAPZ_H */

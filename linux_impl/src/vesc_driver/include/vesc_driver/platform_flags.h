#ifndef PLAT_FLAGS_H_
#define PLAT_FLAGS_H_

#define PLATFORM_IS_LINUX (!defined(TM4C123GH6PM) && !defined(TM4C123GH6PZ) && !defined(TM4C123GH6PGE))

#if !(PLATFORM_IS_LINUX)
#error "Make sure you're prepared to handle this..."
#endif

#endif // PLAT_FLAGS_H_
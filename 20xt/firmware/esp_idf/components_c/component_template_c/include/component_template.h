// Should use both types of header guards
#pragma once
#ifndef _COMPONENT_TEMPLATE_H_
#define _COMPONENT_TEMPLATE_H_

// Need to define functions as extern if compiling for C++
#ifdef __cplusplus
extern "C" {
#endif

void comp_temp_func(void);

// Need to define functions as extern if compiling for C++
#ifdef __cplusplus
}
#endif

#endif
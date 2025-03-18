#pragma once

// Special include file for AsyncHTTPRequest_Teensy41 library
// This prevents multiple definition errors by ensuring implementation
// is only included in one translation unit

#ifndef HTTP_IMPLEMENTATION
// When included without implementation, just get the declarations
#define ASYNC_HTTP_REQUEST_TEENSY41_IMPL_H  // Prevent implementation inclusion
#include <AsyncHTTPRequest_Teensy41.h>
#undef ASYNC_HTTP_REQUEST_TEENSY41_IMPL_H
#else
// When included with implementation, get everything
#include <AsyncHTTPRequest_Teensy41.h>
#endif
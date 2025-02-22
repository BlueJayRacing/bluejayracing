#include "verify_psram.hpp"
#include <unity.h>
#include <SPI.h>
#include <SD.h>

extern "C" uint8_t external_psram_size;

uint32_t *memory_begin, *memory_end;

void test_check_psram()
{
	pinMode(13, OUTPUT);
	uint8_t size = external_psram_size;

	if (size == 0) return;

	memory_begin = (uint32_t *)(0x70000000);
	memory_end = (uint32_t *)(0x70000000 + size * 1048576);
	
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0x5A698421));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(2976674124ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(1438200953ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(3413783263ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(1900517911ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(1227909400ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(276562754ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(146878114ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(615545407ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(110497896ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(74539250ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(4197336575ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(2280382233ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(542894183ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(3978544245ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(2315909796ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(3736286001ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(2876690683ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(215559886ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(539179291ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(537678650ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(4001405270ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(2169216599ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(4036891097ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(1535452389ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(2959727213ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(4219363395ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(1036929753ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(2125248865ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(3177905864ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(2399307098ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(3847634607ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(27467969ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(520563506ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(381313790ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(4174769276ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(3932189449ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(4079717394ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(868357076ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(2474062993ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(1502682190ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(2471230478ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(85016565ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(1427530695ul));
	TEST_ASSERT_EQUAL(true, check_lfsr_pattern(1100533073ul));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0x55555555));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0x33333333));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0x0F0F0F0F));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0x00FF00FF));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0x0000FFFF));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0xAAAAAAAA));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0xCCCCCCCC));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0xF0F0F0F0));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0xFF00FF00));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0xFFFF0000));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0xFFFFFFFF));
	TEST_ASSERT_EQUAL(true, check_fixed_pattern(0x00000000));
}


// fill the entire RAM with a fixed pattern, then check it
bool check_fixed_pattern(uint32_t pattern)
{
	volatile uint32_t *p;
	for (p = memory_begin; p < memory_end; p++) {
		*p = pattern;
	}
	arm_dcache_flush_delete((void *)memory_begin,
		(uint32_t)memory_end - (uint32_t)memory_begin);
	for (p = memory_begin; p < memory_end; p++) {
		uint32_t actual = *p;
		if (actual != pattern) return false;
	}
	return true;
}

// fill the entire RAM with a pseudo-random sequence, then check it
bool check_lfsr_pattern(uint32_t seed)
{
	volatile uint32_t *p;
	uint32_t reg;

	Serial.printf("testing with pseudo-random sequence, seed=%u\n", seed);
	reg = seed;
	for (p = memory_begin; p < memory_end; p++) {
		*p = reg;
		for (int i=0; i < 3; i++) {
			// https://en.wikipedia.org/wiki/Xorshift
			reg ^= reg << 13;
			reg ^= reg >> 17;
			reg ^= reg << 5;
		}
	}
	arm_dcache_flush_delete((void *)memory_begin,
		(uint32_t)memory_end - (uint32_t)memory_begin);
	reg = seed;
	for (p = memory_begin; p < memory_end; p++) {
		uint32_t actual = *p;
		if (actual != reg) return false;
		//Serial.printf(" reg=%08X\n", reg);
		for (int i=0; i < 3; i++) {
			reg ^= reg << 13;
			reg ^= reg >> 17;
			reg ^= reg << 5;
		}
	}
	return true;
}
#include "helper.h"

#include <math.h>
#include <stdbool.h>

void helper_append_int16(char* buffer, short number, int *index) 
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void helper_append_uint16(char* buffer, unsigned short number, int *index) 
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void helper_append_int32(char* buffer, int number, int *index) 
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void helper_append_uint32(char* buffer, unsigned int number, int *index) 
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void helper_append_int64(char* buffer, long long number, int *index) 
{
	buffer[(*index)++] = number >> 56;
	buffer[(*index)++] = number >> 48;
	buffer[(*index)++] = number >> 40;
	buffer[(*index)++] = number >> 32;
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void helper_append_uint64(char* buffer, unsigned long long number, int *index) 
{
	buffer[(*index)++] = number >> 56;
	buffer[(*index)++] = number >> 48;
	buffer[(*index)++] = number >> 40;
	buffer[(*index)++] = number >> 32;
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

short helper_get_int16(const char *buffer, int *index) 
{
	short res =	((unsigned short) buffer[*index]) << 8 |
					((unsigned short) buffer[*index + 1]);
	*index += 2;
	return res;
}

unsigned short helper_get_uint16(const char *buffer, int *index) 
{
	unsigned short res = 	((unsigned short) buffer[*index]) << 8 |
					((unsigned short) buffer[*index + 1]);
	*index += 2;
	return res;
}

int helper_get_int32(const char *buffer, int *index) 
{
	int res =	((unsigned int) buffer[*index]) << 24 |
					((unsigned int) buffer[*index + 1]) << 16 |
					((unsigned int) buffer[*index + 2]) << 8 |
					((unsigned int) buffer[*index + 3]);
	*index += 4;
	return res;
}

unsigned int helper_get_uint32(const char *buffer, int *index) 
{
	unsigned int res =	((unsigned int) buffer[*index]) << 24 |
					((unsigned int) buffer[*index + 1]) << 16 |
					((unsigned int) buffer[*index + 2]) << 8 |
					((unsigned int) buffer[*index + 3]);
	*index += 4;
	return res;
}

long long helper_get_int64(const char *buffer, int *index) 
{
	long long res =	((unsigned long long) buffer[*index]) << 56 |
					((unsigned long long) buffer[*index + 1]) << 48 |
					((unsigned long long) buffer[*index + 2]) << 40 |
					((unsigned long long) buffer[*index + 3]) << 32 |
					((unsigned long long) buffer[*index + 4]) << 24 |
					((unsigned long long) buffer[*index + 5]) << 16 |
					((unsigned long long) buffer[*index + 6]) << 8 |
					((unsigned long long) buffer[*index + 7]);
	*index += 8;
	return res;
}

unsigned long long helper_get_uint64(const char *buffer, int *index) 
{
	unsigned long long res =	((unsigned long long) buffer[*index]) << 56 |
					((unsigned long long) buffer[*index + 1]) << 48 |
					((unsigned long long) buffer[*index + 2]) << 40 |
					((unsigned long long) buffer[*index + 3]) << 32 |
					((unsigned long long) buffer[*index + 4]) << 24 |
					((unsigned long long) buffer[*index + 5]) << 16 |
					((unsigned long long) buffer[*index + 6]) << 8 |
					((unsigned long long) buffer[*index + 7]);
	*index += 8;
	return res;
}

void helper_append_float16(char* buffer, float number, float scale, int *index) 
{
    helper_append_int16(buffer, (short)(number * scale), index);
}

void helper_append_float32(char* buffer, float number, float scale, int *index) 
{
    helper_append_int32(buffer, (int)(number * scale), index);
}

void helper_append_float32_auto(char* buffer, float number, int *index) 
{
    #if 0
	int e = 0;
	float sig = frexpf(number, &e);
	float sig_abs = fabsf(sig);
	unsigned int sig_i = 0;

	if (sig_abs >= 0.5) {
		sig_i = (unsigned int)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
		e += 126;
	}

	unsigned int res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
	if (sig < 0) {
		res |= 1 << 31;
	}

	helper_append_uint32(buffer, res, index);
    #endif
}

float helper_get_float16(const char *buffer, float scale, int *index) 
{
    return (float)helper_get_int16(buffer, index) / scale;
}

float helper_get_float32(const char *buffer, float scale, int *index) 
{
    return (float)helper_get_int32(buffer, index) / scale;
}

float helper_get_float32_auto(const char *buffer, int *index) 
{
    #if 0
	unsigned int res = helper_get_uint32(buffer, index);

	int e = (res >> 23) & 0xFF;
	unsigned int  sig_i = res & 0x7FFFFF;

	char neg = res & (1 << 31);

	float sig = 0.0;
	if (e != 0 || sig_i != 0) 
    {
		sig = (float)sig_i / (8388608.0 * 2.0) + 0.5;
		e -= 126;
	}

	if (neg) 
    {
		sig = -sig;
	}

	return ldexpf(sig, e);
    #endif
}

void * helper_memcpy(void *dest, const void *src, unsigned int len)
{
  char *d = dest;
  const char *s = src;
  while (len--)
    *d++ = *s++;
  return dest;
}


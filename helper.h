#ifndef HELPER_H_
#define HELPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>


void helper_append_int16(char* buffer, int16_t number, int32_t *index);
void helper_append_uint16(char* buffer, uint16_t number, int32_t *index);
void helper_append_int32(char* buffer, int32_t number, int32_t *index);
void helper_append_uint32(char* buffer, uint32_t number, int32_t *index);
void helper_append_int64(char* buffer, int64_t number, int32_t *index);
void helper_append_uint64(char* buffer, uint64_t number, int32_t *index);
int16_t helper_get_int16(const char *buffer, int32_t *index);
uint16_t helper_get_uint16(const char *buffer, int32_t *index);
int32_t helper_get_int32(const char *buffer, int32_t *index);
uint32_t helper_get_uint32(const char *buffer, int32_t *index);
int64_t helper_get_int64(const char *buffer, int32_t *index);
uint64_t helper_get_uint64(const char *buffer, int32_t *index);

void helper_append_float16(char* buffer, float number, float scale, int32_t *index);
void helper_append_float32(char* buffer, float number, float scale, int32_t *index);
void helper_append_float32_auto(char* buffer, float number, int32_t *index);

float helper_get_float16(const char *buffer, float scale, int *index);
float helper_get_float32(const char *buffer, float scale, int *index);
float helper_get_float32_auto(const char *buffer, int *index);

void * helper_memcpy(void *dest, const void *src, size_t len);


#ifdef __cplusplus
}
#endif

#endif /* HELPER_H_ */
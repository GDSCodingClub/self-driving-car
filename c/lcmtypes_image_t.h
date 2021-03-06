/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by LCM
 **/

#include <stdint.h>
#include <stdlib.h>
#include "lcm_coretypes.h"

#ifndef _lcmtypes_image_t_h
#define _lcmtypes_image_t_h

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _lcmtypes_image_t lcmtypes_image_t;
struct _lcmtypes_image_t
{
    int64_t    utime;
    int16_t    width;
    int16_t    height;
    int16_t    stride;
    int32_t    pixelformat;
    int32_t    size;
    uint8_t    *image;
};
 
lcmtypes_image_t   *lcmtypes_image_t_copy(const lcmtypes_image_t *p);
void lcmtypes_image_t_destroy(lcmtypes_image_t *p);

int  lcmtypes_image_t_encode(void *buf, int offset, int maxlen, const lcmtypes_image_t *p);
int  lcmtypes_image_t_decode(const void *buf, int offset, int maxlen, lcmtypes_image_t *p);
int  lcmtypes_image_t_decode_cleanup(lcmtypes_image_t *p);
int  lcmtypes_image_t_encoded_size(const lcmtypes_image_t *p);

// LCM support functions. Users should not call these
int64_t __lcmtypes_image_t_get_hash(void);
int64_t __lcmtypes_image_t_hash_recursive(const __lcm_hash_ptr *p);
int     __lcmtypes_image_t_encode_array(void *buf, int offset, int maxlen, const lcmtypes_image_t *p, int elements);
int     __lcmtypes_image_t_decode_array(const void *buf, int offset, int maxlen, lcmtypes_image_t *p, int elements);
int     __lcmtypes_image_t_decode_array_cleanup(lcmtypes_image_t *p, int elements);
int     __lcmtypes_image_t_encoded_array_size(const lcmtypes_image_t *p, int elements);
int     __lcmtypes_image_t_clone_array(const lcmtypes_image_t *p, lcmtypes_image_t *q, int elements);

#ifdef __cplusplus
}
#endif

#endif

/* texture.c:
 *
 * Convenience functions for uploading OpenGL textures and drawing them.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <glib.h>

#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#include <GL/glext.h>

#include "texture.h"

#define err(args...) fprintf(stderr, args)

struct _GLUtilTexture {
    int width, height;
    GLenum target;
    GLint int_format;
    GLuint texname;
    GLint interp_mode;

    GLuint texc_width;
    GLuint texc_height;

    GLuint pbo;
    int use_pbo;
    int max_data_size;
};
    
GLUtilTexture *
glutil_texture_new (int width, int height, int max_data_size)
{
    GLUtilTexture * t;

    t = malloc (sizeof (GLUtilTexture));
    memset (t, 0, sizeof (GLUtilTexture));

    int has_non_power_of_two = 0;
    int has_texture_rectangle = 0;
    int has_pbo = 0;

    const char * extstr = (const char *) glGetString (GL_EXTENSIONS);
    gchar ** exts = g_strsplit (extstr, " ", 0);
    int i;
    for (i = 0; exts[i]; i++) {
        gchar * ext = exts[i];
        if (!strcmp (ext, "GL_ARB_texture_non_power_of_two"))
            has_non_power_of_two = 1;
        if (!strcmp (ext, "GL_ARB_texture_rectangle"))
            has_texture_rectangle = 1;
        if (!strcmp (ext, "GL_ARB_pixel_buffer_object"))
            has_pbo = 1;
    }
    g_strfreev (exts);

//    printf ("%s:%d - %d %d %d\n", __FILE__, __LINE__,
//            has_non_power_of_two, has_texture_rectangle,
//            has_pbo
//            );

    t->use_pbo = has_pbo;
    t->int_format = GL_RGBA8;
    t->interp_mode = GL_LINEAR;
    t->width = width;
    t->height = height;

    if (has_non_power_of_two) {
        t->target = GL_TEXTURE_2D;
        t->texc_width = 1;
        t->texc_height = 1;
    }
    else if (has_texture_rectangle) {
        t->target = GL_TEXTURE_RECTANGLE_ARB;
        t->texc_width = width;
        t->texc_height = height;
    }
    else {
        fprintf (stderr, "Error: GL supports neither non-power-of-two nor "
                "texture-rectangle\n");
        free (t);
        return NULL;
    }

    glGenTextures (1, &t->texname);
#if 0
    glBindTexture (t->target, t->texname);

#endif

    if (t->use_pbo) {
        glGenBuffersARB (1, &t->pbo);
        t->max_data_size = max_data_size;
        glBindBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB, t->pbo);
        glBufferDataARB (GL_PIXEL_UNPACK_BUFFER_ARB, t->max_data_size, NULL, 
                GL_STREAM_DRAW);
        glBindBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB, 0);
    }

    return t;
}

void
glutil_texture_free (GLUtilTexture * t)
{
    glDeleteTextures (1, &t->texname);
    if (t->pbo) {
        glDeleteBuffersARB (1, &t->pbo);
    }
    free (t);
}

void
glutil_texture_draw (GLUtilTexture * t)
{
    glEnable (t->target);
    glBindTexture (t->target, t->texname);
    glBegin (GL_QUADS);
    glTexCoord2i (0,0);
    glVertex2i (0,0);
    glTexCoord2i (0,t->texc_height);
    glVertex2i (0,1);
    glTexCoord2i (t->texc_width,t->texc_height);
    glVertex2i (1, 1);
    glTexCoord2i (t->texc_width,0);
    glVertex2i (1,0);
    glEnd ();
    glBindTexture (t->target, 0);
    glDisable (t->target);
}
void
glutil_texture_draw_coords (GLUtilTexture * t, 
        double x0, double y0, double x1, double y1, 
        double x2, double y2, double x3, double y3)
{
    glEnable (t->target);
    glBindTexture (t->target, t->texname);
    glBegin (GL_QUADS);
    glTexCoord2i (0,0);
    glVertex2i (x0,y0);
    glTexCoord2i (0,t->texc_height);
    glVertex2i (x1,y1);
    glTexCoord2i (t->texc_width,t->texc_height);
    glVertex2i (x2,y2);
    glTexCoord2i (t->texc_width,0);
    glVertex2i (x3,y3);
    glEnd ();
    glBindTexture (t->target, 0);
    glDisable (t->target);
}

static inline int _bits_per_channel (GLenum type)
{
    switch (type) {
    case GL_UNSIGNED_BYTE:
    case GL_BYTE:
        return 8;
    case GL_UNSIGNED_SHORT:
    case GL_SHORT:
        return 16;
    case GL_UNSIGNED_INT:
    case GL_INT:
    case GL_FLOAT:
        return 32;
    default:
        return 0;
    }
// GL_UNSIGNED_BYTE_3_3_2, GL_UNSIGNED_BYTE_2_3_3_REV,
// GL_UNSIGNED_SHORT_5_6_5, GL_UNSIGNED_SHORT_5_6_5_REV,
// GL_UNSIGNED_SHORT_4_4_4_4, GL_UNSIGNED_SHORT_4_4_4_4_REV,
// GL_UNSIGNED_SHORT_5_5_5_1, GL_UNSIGNED_SHORT_1_5_5_5_REV,
// GL_UNSIGNED_INT_8_8_8_8, GL_UNSIGNED_INT_8_8_8_8_REV,
// GL_UNSIGNED_INT_10_10_10_2, and GL_UNSIGNED_INT_2_10_10_10_REV
}

static inline int _num_channels (GLenum format) {
    switch (format) {
        case GL_LUMINANCE:
        case GL_ALPHA:
            return 1;
        case GL_RGB:
        case GL_BGR:
            return 3;
        case GL_RGBA:
        case GL_BGRA:
            return 4;
        default:
            return 0;
    }
}

int
glutil_texture_upload (GLUtilTexture * t, GLenum format,
        GLenum type, int stride, void * data)
{
    if (t->use_pbo && (stride * t->height) > t->max_data_size) {
        fprintf (stderr, 
                "Error: glutil_texture buffer (%d bytes) too small for "
                "texture (%d bytes)\n", t->max_data_size, stride * t->height);
        return -1;
    }
    if (!data && !t->use_pbo) {
        fprintf (stderr, "Error: glutil_texture data is NULL\n");
        return -1;
    }

    int bits_per_pixel = _bits_per_channel (type) * _num_channels (format);
    if (! bits_per_pixel) {
        fprintf (stderr, "Error: glutil_texture unsupported format/type pair\n");
        return -1;
    }

    glBindTexture (t->target, t->texname);

    glTexParameterf (t->target, GL_TEXTURE_MAG_FILTER, t->interp_mode);
    glTexParameterf (t->target, GL_TEXTURE_MIN_FILTER, t->interp_mode);

    if (stride % 2) {
        glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    } else if (stride % 4) {
        glPixelStorei (GL_UNPACK_ALIGNMENT, 2);
    } else {
        glPixelStorei (GL_UNPACK_ALIGNMENT, 4);
    }

    glPixelStorei (GL_UNPACK_ROW_LENGTH, stride * 8 / bits_per_pixel);
    if (t->use_pbo) {

        glBindBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB, t->pbo);

        /* By setting data to NULL, we skip the memcpy and just re-upload
         * from the buffer object.  This can be useful to re-upload with
         * different PixelTransfer settings. */
        if (data) {
            uint8_t *buffer_data = 
                (uint8_t*) glMapBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB,
                        GL_WRITE_ONLY);
            if (!buffer_data) {
                glPixelStorei (GL_UNPACK_ROW_LENGTH, 0);
                glBindTexture (t->target, 0);
                return -1;
            }
            memcpy (buffer_data, data, stride * t->height);
            glUnmapBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB);
        }

        glTexImage2D (t->target, 0, t->int_format, t->width, t->height, 0,
                format, type, 0);

        glBindBufferARB (GL_PIXEL_UNPACK_BUFFER_ARB, 0);
    } else {
       glTexImage2D (t->target, 0, t->int_format, t->width, t->height, 0,
                format, type, data);
    }
    glPixelStorei (GL_UNPACK_ROW_LENGTH, 0);
    glBindTexture (t->target, 0);
    return 0;
}

void
glutil_texture_set_interp (GLUtilTexture * t, GLint nearest_or_linear)
{
    t->interp_mode = nearest_or_linear;
}

void 
glutil_texture_set_internal_format (GLUtilTexture *t, GLenum fmt)
{
    t->int_format = fmt;
}

#if 0
void
gl_util_new_sub_frame (GLUtil * gu, void * data, GLenum format, GLenum type,
        int row_length,
        int x, int y, int w, int h)
{
    glBindTexture (gu->target, gu->texname);
    glPixelStorei (GL_UNPACK_ROW_LENGTH, row_length);
    glTexSubImage2D (gu->target, 0, x, y, w, h, format, type, data);
    glPixelStorei (GL_UNPACK_ROW_LENGTH, 0);
    glBindTexture (gu->target, 0);
}

#endif

int glutil_texture_get_width (GLUtilTexture *t) { return t->width; }

int glutil_texture_get_height (GLUtilTexture *t) { return t->height; }


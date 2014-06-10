// file: ppm.c
// auth: Albert Huang <albert@csail.mit.edu>
// date: July 26, 2006
// desc: reads a PPM file into a memory buffer
//
// $Id: ppm.c 288 2006-08-01 03:30:30Z mwalter $

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "ppm.h"

static int
skip_whitespace_and_comments (FILE *fp)
{
    int c = fgetc(fp);
    while (isspace(c) && c != EOF && c == '#') {
        do {
            c = fgetc (fp);
        } while (isspace(c) && c != EOF);

        if (c == EOF) {
            fprintf(stderr, "unexpected EOF\n"); return EOF;
        }
        // ignore comments
        if (c == '#') {
            do {
                c = fgetc(fp);
            } while (c != EOF && c != '\n');
            if (c == EOF) {
                fprintf(stderr, "unexpected EOF\n"); return EOF;
            }
            c = fgetc(fp);
        }
    }
    ungetc(c, fp);
    return 0;
}

static int
read_header (FILE *fp, const char *magic, int *width, int *height, 
        int *maxval)
{
    char m[3] = { 0 };
    fread (m, sizeof(m)-1, 1, fp);
    if (strcmp(m, magic)) {
        fprintf(stderr, "bad magic [%s]\n", m); return -1;
    }
    if (0 != skip_whitespace_and_comments(fp)) return -1;
    if (1 != fscanf(fp, "%d", width)) return -1;
    if (0 != skip_whitespace_and_comments(fp)) return -1;
    if (1 != fscanf(fp, "%d", height)) return -1;
    if (0 != skip_whitespace_and_comments(fp)) return -1;
    if (1 != fscanf(fp, "%d", maxval)) return -1;
    if (EOF == fgetc(fp)) return -1;
    return 0;
}

int ppm_read (FILE *fp, uint8_t **pixels, 
        int *width, int *height, 
        int *rowstride)
{
    int maxval;
    int i;
    int nread;
    int w, h, rs;
    
    if (0 != read_header (fp, "P6", &w, &h, &maxval)) {
        fprintf(stderr, "that doesn't look like a PPM file!\n");
        return -1;
    }

    rs = w * 3;
    rs += rs % 4; // align each row on a 32-bit boundary

    *pixels = (unsigned char*) calloc (h, rs);
    for (i=0; i<h; i++) {
        nread = fread (*pixels + i*rs, w * 3, 1, fp);
        if (1 != nread) {
            perror("fread ppm");
            return -1;
        }
    }

    *height = h;
    *width = w;
    *rowstride = rs;

    return 0;
}

int ppm_write (FILE *fp, const uint8_t *pixels,
        int width, int height, 
        int rowstride)
{
    fprintf(fp, "P6 %d %d %d\n", width, height, 255);
    int i, count;
    for (i=0; i<height; i++){
        count = fwrite(pixels + i*rowstride, width*3, 1, fp);
        if (1 != count) return -1;
    }
    return 0;
}

int ppm_write_bottom_up (FILE *fp, uint8_t *pixels,
        int width, int height, 
        int rowstride)
{
    fprintf(fp, "P6 %d %d %d\n", width, height, 255);
    int i, count;
    for (i=height-1; i>=0; i--){
        count = fwrite(pixels + i*rowstride, width*3, 1, fp);
        if (1 != count) return -1;
    }
    return 0;
}

int pgm_read (FILE *fp, uint8_t **pixels, 
        int *width, int *height, 
        int *rowstride)
{
    int maxval;
    int i;
    int nread;
    int w, h, rs;
    
    if (0 != read_header (fp, "P5", &w, &h, &maxval)) {
        fprintf(stderr, "that doesn't look like a PGM file!\n");
        return -1;
    }

    rs = w;
    rs += rs % 4; // align each row on a 32-bit boundary

    *pixels = (unsigned char*) calloc (h, rs);
    for (i=0; i<h; i++) {
        nread = fread (*pixels + i*rs, 1, w, fp);
        if (w != nread) {
            perror("fread pgm");
            fprintf(stderr, "only read %d bytes (expected %d)\n", 
                    nread, w);
            return -1;
        }
    }

    *height = h;
    *width = w;
    *rowstride = rs;

    return 0;
}

int pgm_write (FILE *fp, const uint8_t *pixels,
        int width, int height, 
        int rowstride)
{
    fprintf(fp, "P5\n%d\n%d\n%d\n", width, height, 255);
    int i, count;
    for (i=0; i<height; i++){
        count = fwrite(pixels + i*rowstride, width, 1, fp);
        if (1 != count) return -1;
    }
    return 0;
}

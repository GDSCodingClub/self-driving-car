// file: example2-images.c
//
// Given a log file, extract the camera thumbnail images and save them to
// individual JPEG files.  Files are named by the acquiring camera and the time
// of acquisition.  Times are given in microseconds since 00:00:00 January 1,
// 1970 UTC.

#include <stdio.h>
#include <string.h>

#include "eventlog.h"
#include "lcmtypes_image_t.h"

int main(int argc, char **argv)
{
    if(argc < 2) {
        fprintf(stderr, "usage: example2-images <logfile>\n");
        return 1;
    }

    lcm_eventlog_t *log = lcm_eventlog_create(argv[1], "r");
    if(!log) {
        fprintf(stderr, "error opening log file\n");
        return 1;
    }

    lcm_eventlog_event_t *event = lcm_eventlog_read_next_event(log);
    while(event) {
        if(!strncmp(event->channel, "CAM_THUMB", 9)) {
            lcmtypes_image_t img;
            lcmtypes_image_t_decode(event->data, 0, event->datalen, &img);

            char fname[80];
            sprintf(fname, "%s-%lld.jpg", event->channel, (long long)img.utime);
            FILE *fp = fopen(fname, "wb");
            int status = fwrite(img.image, img.size, 1, fp);
            if(status != 1) {
                perror("fwrite");
                return 1;
            }
            fclose(fp);
            printf("%s\n", fname);
            lcmtypes_image_t_decode_cleanup(&img);
        }

        lcm_eventlog_free_event(event);
        event = lcm_eventlog_read_next_event(log);
    }

    lcm_eventlog_destroy(log);

    return 0;
}

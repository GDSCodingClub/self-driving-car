CFLAGS:=-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE 

objects:=eventlog.o \
   lcmtypes_gps_to_local_t.o \
   lcmtypes_pose_t.o \
   lcmtypes_image_t.o \
   lcmtypes_velodyne_t.o \
   lcmtypes_laser_t.o \
   config.o \
   rotations.o \
   math_util.o \
   velodyne.o \
   config_util.o \
   camtrans.o

examples:=example1-poses \
	example2-images \
	example3-gps \
	example4-velodyne \
	example5-sick

all: $(examples)

example1-poses: example1-poses.o $(objects)
	gcc $(CFLAGS) -o $@ $^ -lm

example2-images: example2-images.o $(objects)
	gcc $(CFLAGS) -o $@ $^ -lm

example3-gps: example3-gps.o $(objects)
	gcc $(CFLAGS) -o $@ $^ -lm

example4-velodyne: example4-velodyne.o $(objects)
	gcc $(CFLAGS) -o $@ $^ -lm

example5-sick: example5-sick.o $(objects)
	gcc $(CFLAGS) -o $@ $^ -lm

%.o: %.c
	gcc -g -c -Wall -o $@ $< 

clean:
	rm -f $(examples) *.o

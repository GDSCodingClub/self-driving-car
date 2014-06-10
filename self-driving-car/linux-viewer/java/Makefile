lcmtypes:=$(shell ls ../lcmtypes/*.lcm)
basenames:=$(basename $(notdir $(lcmtypes)))
java_lcmtypes:=$(basenames:%=src/lcmtypes/%.java)

CLASSPATH_LCM:=`pkg-config --variable=classpath lcm-java`

CLASSPATH:=$(shell pwd)/src:$(CLASSPATH_LCM)

mitdgc_spy=../bin/mitdgc-spy

all: $(java_lcmtypes) $(mitdgc_spy)
	CLASSPATH=$(CLASSPATH) ant

src/lcmtypes/%.java: ../lcmtypes/%.lcm
	lcm-gen $^ -j --jpath=src --lazy

$(mitdgc_spy):
	@echo > $@ "#!/bin/sh"
	@echo >> $@ "CLASSPATH=\"$(CLASSPATH_LCM):$(shell pwd)/mitdgc.jar\" lcm-spy"
	chmod 755 $@

clean:
	rm -f $(java_lcmtypes) $(mitdgc_spy)
	ant clean

CFLAGS	=	-O2 -DTARGET_FLOAT32_IS_FIXED -IInclude 

TARGETS =	android

LIB = 		libbox2d.a

SOURCES	=	$(wildcard Source/*/*.cpp) $(wildcard Source/*/*/*.cpp) 

all:	$(TARGETS)

define PLATFORM_rules
$(1)_SOURCES = $$(SOURCES)
$(1)_OBJS	= $$($(1)_SOURCES:Source/%.cpp=obj/$(1)/%.o)
$(1)_LIB	= lib/$(1)/$$(LIB)

$(1):	$$($(1)_LIB)

$$($(1)_LIB):	$$($(1)_OBJS)
	@mkdir -p $$(dir $$@)
	$$(TOOL_PREFIX)ar rc $$@ $$^

obj/$(1)/%.o: Source/%.cpp
	mkdir -p $$(dir $$@)
	$$(TOOL_PREFIX)g++ $$(CFLAGS) -DDAS_PLATFORM_$(1) -MD -o $$@ -c $$< 

-include $$($(1)_OBJS:%.o=%.d)
endef

$(foreach platform,$(TARGETS),$(eval $(call PLATFORM_rules,$(platform))))

ANDROID_TOOL_PREFIX = C:/android/android-ndk-1.5_r1/build/prebuilt/windows/arm-eabi-4.2.1/bin/arm-eabi-
ANDROID_NDK_BASE = /android/android-ndk-1.5_r1
ANDROID_CFLAGS = -march=armv5te -mtune=xscale -msoft-float -fpic -mthumb-interwork \
	-ffunction-sections -funwind-tables -fstack-protector -fno-short-enums \
	-fno-exceptions -fno-rtti \
	-D__ARM_ARCH_5__ -D__ARM_ARCH_5T__ -D__ARM_ARCH_5E__ -D__ARM_ARCH_5TE__ -DANDROID -O2 -DNDEBUG -g \
	-I$(ANDROID_NDK_BASE)/build/platforms/android-1.5/arch-arm/usr/include

android:	TOOL_PREFIX = $(ANDROID_TOOL_PREFIX)
android:	CFLAGS += $(ANDROID_CFLAGS)

clean:
	rm -fr obj lib

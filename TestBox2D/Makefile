CFLAGS	= -O2 -DTARGET_FLOAT32_IS_FIXED -I../box2d/Include
LIBS	= -L../box2d/lib/android -lbox2d
LIBNAME = libtest.so

SOURCES = $(wildcard native/*.cpp)
OBJS	= $(SOURCES:native/%.cpp=obj/%.o)
SYMS	= obj/$(LIBNAME)
LIB		= libs/armeabi/$(LIBNAME)

all:	$(LIB)

ANDROID_NDK_BASE = /cygdrive/c/android/android-ndk-1.5_r1
ANDROID_TOOL_PREFIX = $(ANDROID_NDK_BASE)/build/prebuilt/windows/arm-eabi-4.2.1/bin/arm-eabi-
ANDROID_CFLAGS = -march=armv5te -mtune=xscale -msoft-float -fpic -mthumb-interwork \
	-ffunction-sections -funwind-tables -fstack-protector -fno-short-enums \
	-fno-exceptions -fno-rtti \
	-D__ARM_ARCH_5__ -D__ARM_ARCH_5T__ -D__ARM_ARCH_5E__ -D__ARM_ARCH_5TE__ -DANDROID -O2 -DNDEBUG -g \
	-I$(ANDROID_NDK_BASE)/build/platforms/android-1.5/arch-arm/usr/include

$(LIB):	$(SYMS)
	@mkdir -p $(dir $@)
	$(ANDROID_TOOL_PREFIX)strip --strip-debug $^ -o $@

$(SYMS):	$(OBJS)
	@mkdir -p $(dir $@)
	$(ANDROID_TOOL_PREFIX)ld \
		-nostdlib -shared -Bsymbolic --no-undefined \
		-o $(SYMS) $(OBJS) $(LIBS) \
		-L$(ANDROID_NDK_BASE)/build/platforms/android-1.5/arch-arm/usr/lib \
		-llog -lc -lstdc++ -lm \
		$(ANDROID_NDK_BASE)/build/prebuilt/windows/arm-eabi-4.2.1/lib/gcc/arm-eabi/4.2.1/interwork/libgcc.a \

obj/%.o:	native/%.cpp
	@mkdir -p $(dir $@)
	$(ANDROID_TOOL_PREFIX)g++ $(CFLAGS) $(ANDROID_CFLAGS) -MD -o $@ -c $<

-include $(OBJS:%.o=%.d)

clean:
	rm -fr obj libs

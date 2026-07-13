Import("env")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(
        "$OBJCOPY -O binary $BUILD_DIR/${PROGNAME}.elf $BUILD_DIR/firmware.bin",
        "Building $BUILD_DIR/firmware.bin",
    ),
)

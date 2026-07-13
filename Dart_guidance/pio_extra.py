Import("env")


def _remove_flag(flag_list, prefix):
    return [item for item in flag_list if item != prefix and not item.startswith(prefix + "=")]


for key in ("ASFLAGS", "CCFLAGS", "CXXFLAGS", "LINKFLAGS"):
    flags = env.get(key, [])
    flags = _remove_flag(flags, "-mfloat-abi")
    flags = _remove_flag(flags, "-mfpu")
    flags = _remove_flag(flags, "-mcpu")
    flags = _remove_flag(flags, "-mthumb")
    flags.extend([
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16",
    ])
    env.Replace(**{key: flags})

#!/usr/bin/env python

#
# SOURCE : https://github.com/ankostis/Freematics/blob/dev/firmware_v5/telelogger/patchappinfos.py
#

"""
A *platformio* script to engrave app-infos by patching espressif32 images.
**Rational**
This engraving is necessary for discerning images/pertitions when OTA-upgrading,
since *platformio* does not preserve `espidf` framework behavior when
`arduino` framework is also enabled.
**Image Infos**
The infos extracted by `_collect_app_infos(env)` try to mimic ESP-IDF behavior,
as listed below (in precedance order):
- **project-name:**
  - no patching if `custom_appinfos_patch_appname` custom option is truthy
    (emulates a *negated* [`CONFIG_APP_EXCLUDE_PROJECT_NAME_VAR` cppdefine
    from ESP_IDF](https://docs.espressif.com/projects/esp-idf/en/v4.4.1/esp32/api-reference/kconfig.html#config-app-exclude-project-name-var))
    [default: true]
  - `custom_prog_name` custom option
    (emulates `PROJECT_NAME` cppdefine from ESP-IDF)
  - git-relative dirs of project root(`${PROJECT_SRC_DIR}`)
    (deviates from ESP-IDF)
  - last dir-name of project root (`${PROJECT_SRC_DIR}`)
    (deviates from ESP-IDF)
  - `"firmware"`
  - appends `{user}@{host}` suffix into *project-name*  if `custom_appinfos_patch_builder`
    custom option is truthy
    [default: true]
- **project-version:**
  - no patching if `custom_appinfos_patch_appver` custom option is truthy
    (emulates a *negated* [`CONFIG_APP_EXCLUDE_PROJECT_VER_VAR` cppdefine
    from ESP_IDF](https://docs.espressif.com/projects/esp-idf/en/v4.4.1/esp32/api-reference/kconfig.html#config-app-exclude-project-ver-var))
    [default: true]
  - `custom_prog_version` custom option
    (emulates `PROJECT_VERSION` cppdefine from ESP-IDF)
  - `${PROJECT_PATH}/version.txt` file in project root
  - git describe (mimics ESP-IDF)
  - `"1"` (mimics ESP-IDF)
  - does not respect ESP-IDF's *Kconfigs*:
    - `CONFIG_APP_PROJECT_VER_FROM_CONFIG`
    - `CONFIG_APP_PROJECT_VER`
    - see: https://docs.espressif.com/projects/esp-idf/en/v4.4.1/esp32/api-reference/system/system.html#app-version
- **build date/time:**
  - no patching if `custom_appinfos_patch_timestamp` custom option is truthy
    (emulates [the `CONFIG_APP_COMPILE_TIME_DATE` cppdefine
    from ESP_IDF](https://docs.espressif.com/projects/esp-idf/en/v4.4.1/esp32/api-reference/kconfig.html#config-app-exclude-project-ver-var))
    [default: true]
  - `custom_build_date/custom_build_time` custom options
    (deviates ESP-IDF);
  - Python's `datetime.now()` (taken *after* firmware has been build,
    at the time the image is *post-processed*)
  - does not respect ESP-IDF's *Kconfigs*.
- Boolean config-options are considered false if defined but
  are matched,case-insensitively, in :data:`FALSE_VALUES` by :func:`_get_bool_option()`.
- Patching-offsets were calculated from:
  https://docs.espressif.com/projects/esp-idf/en/v4.4.1/esp32/api-reference/system/app_image_format.html#_CPPv414esp_app_desc_t
- TODO: mimic more behavior from ESP-IDF (eg Kconfigs).
"""
#%%
import hashlib
import io
import operator
import shutil
import struct
import functools
import subprocess
import sys
import typing
from collections import namedtuple
from pathlib import Path
from datetime import datetime
from typing import BinaryIO, Iterator, Tuple

import progname


#: Git command to show relative-dir of the current project, to use as project-name.
GIT_RELATIVE_DIR_CMD = "git rev-parse --show-prefix".split()
#: values for config-options translated case-insensitively as `False`.
FALSE_VALUES = {*"false off no 0".split()}
#: to convert names into bytes
ENCODING = "utf-8"
#: Where app-name/version & build-date/time are stored in the image?
ESP_APP_DESC_OFFSET = 0x30
#: Start traversing segments while checksuming from this file postion.
FIRST_SEGMENT_OFFSET = 0x18
## Magic constants from `esp_app_format.h`
ESP_IMAGE_HEADER_MAGIC = 0xE9
ESP_APP_DESC_MAGIC_WORD = 0xABCD5432
#: Initial state for the checksum routine.
ESP_CHECKSUM_MAGIC = 0xEF
#: Algo factory for clalculating the image's new checksum after patching.
FILE_HASHING_ALGO = hashlib.sha256
#: The length (in bytes) of the hash256 at image's EOF.
FILE_HASH_LENGTH = 32

#%%
from platformio.compat import MISSING


def _get_bool_option(env, option: str, default=MISSING) -> bool:
    """
    Translates custom-options in `platform.ini` as booleans
    :param default:
        applies only if `option` undefined (ie missing from `platform.io`),
        translated through :data:`FALSE_VALUES`.
    :return:
        - if undefined (ie `option` missing): `default` if given, exception otherwise
        - if `option` defined but without any value specified: `True`
        - otherwise: the value for `option`.
        If option value (or `default`) matches any of :data:`FALSE_VALUES`
        (case-insensitevly), `False` returned.
    """
    value = env.GetProjectOption(option, default)
    if value == "":  # something like `-Dfoo`
        return True
    return value.lower() not in FALSE_VALUES if isinstance(value, str) else value


def git_relative_dir() -> str:
    try:
        return (
            subprocess.check_output(GIT_RELATIVE_DIR_CMD, universal_newlines=True)
            .strip()
            .strip("/")
        )
    except Exception as ex:
        ## Logging source of each app-ingo
        pass
        # sys.stderr.write(
        #     f"could not read project-name from `{GIT_RELATIVE_DIR_CMD}`"
        #     f" due to {type(ex).__name__}: {ex}\n"
        # )


def get_project_name(env) -> Tuple[str, str]:
    """
    :return: 2-tuple of (project, source-label)
    Note, didn't reuse :func:`get_program_name()` bc app-infos can accomodate
    relative git-path of project-root as `appname`.
    """
    branch = ""

    try:
        branch = subprocess.check_output(
            "git branch --show-current".split(), universal_newlines=True
        ).strip()
    except Exception as ex:
        branch = "UNKNOWN"

    return progname.fallback_get(
        (
            lambda: "unified-un52",
            "custom_option",
        ),
        (lambda: git_relative_dir(), "git_relative_dir"),
        (
            lambda: env.Dir(env.get("PROJECT_SRC_DIR")).get_path_elements()[-1].name,
            "project_dir",
        ),
    )


AppInfos = namedtuple("AppInfos", "appname, appver, date, time")


def _collect_app_infos(env) -> Tuple[AppInfos, AppInfos]:
    bool_option = functools.partial(_get_bool_option, env)
    appname = appver = date = time = None
    appname_src = appver_src = date_src = time_src = None

    if bool_option("custom_appinfos_patch_appname", True):
        appname, appname_src = get_project_name(env)

    ## TODO: parse var-value as False if off/false/no/0.
    if bool_option("custom_appinfos_patch_appver", True):
        appver, appver_src = progname.get_program_ver(env)

    ## TODO: parse var-value as False if off/false/no/0.
    if bool_option("custom_appinfos_patch_timestamp", True):
        date = env.GetProjectOption("custom_build_date", None)
        time = env.GetProjectOption("custom_build_time", None)
        if not date or not time:
            now = datetime.now(datetime.utcnow().astimezone().tzinfo)
            if not date:
                date = now.strftime("%d %b %Y")
                date_src = "python"
            else:
                date_src = "custom_option"
            if not time:
                time = now.strftime("%H:%M:%S%z")
                time_src = "python"
            else:
                time_src = "custom_option"

    return AppInfos(appname, appver, date, time), AppInfos(
        appname_src, appver_src, date_src, time_src
    )


#%%


def chunks(
    fd: BinaryIO, length=None, *, chunk_size=io.DEFAULT_BUFFER_SIZE
) -> Iterator[bytes]:
    """
    :param length:
        the total number of bytes to consume;  if `None`, exhaust stream,
        if negative, yield until that many bytes before EOF.
    :param chunk_size:
        the maximum length of each chunk (the last one maybe shorter)
    :return:
        an iterator over the chunks of the file
    """
    if length is None:
        yield from iter(lambda: fd.read(chunk_size), b"")
    else:
        if length < 0:
            fsize = Path(fd.name).stat().st_size
            length = fsize - fd.tell() + length  # negative `length` added!
        consumed = 0
        for chunk in iter(lambda: fd.read(min(chunk_size, length - consumed)), b""):
            yield chunk
            consumed += len(chunk)


def digest_stream(
    fd: BinaryIO, digester_factory, length, chunk_size=io.DEFAULT_BUFFER_SIZE
) -> bytes:
    """
    :param length:
        how many bytes to digest from the start of the file; if not positive,
        digesting stops that many bytes before EOF (eg 0 means all file).
    """
    digester = digester_factory()
    for chunk in chunks(fd, length, chunk_size=chunk_size):
        digester.update(chunk)
    return digester.digest()


#%%


def checksum_segment(fd, state, segment_ix) -> int:
    size = struct.unpack("<4xI", fd.read(8))[0]
    bdata = fd.read(size)
    if len(bdata) < size:
        raise ValueError(
            f"Premature EOF of segment({segment_ix})@{len(bdata)} != {size}!"
        )
    return functools.reduce(operator.xor, bdata, state)


def align_file_position(f, size):
    """Align the position in the file to the next block of specified size"""
    align = (size - 1) - (f.tell() % size)
    f.seek(align, 1)


def checksum_image(fd: BinaryIO, nsegments: int, patch=None) -> int:
    fd.seek(FIRST_SEGMENT_OFFSET)
    state = ESP_CHECKSUM_MAGIC
    for i in range(nsegments):
        state = checksum_segment(fd, state, i)
        # print(f"  +--seg({i} out of {nsegments}): 0x{state:02x}")
    align_file_position(fd, 16)
    stored = ord(fd.read(1))
    if patch:
        print(f"  +--checksum@0x{fd.tell():08x}): 0x{stored:02x} --> 0x{state:02x}")
        fd.seek(-1, 1)
        fd.write(struct.pack("B", state))
    else:
        if state != stored:
            raise ValueError(
                f"{fd.name}: image's checksum(0x{stored:02x}) != 0x{state:02x}!"
            )

    return state


def hash_image(fd: BinaryIO, patch=None):
    fd.seek(0)
    hash = digest_stream(fd, FILE_HASHING_ALGO, -FILE_HASH_LENGTH)
    assert len(hash) == FILE_HASH_LENGTH, (hash, FILE_HASH_LENGTH)
    # After digesting hash, file positioned immediately before the hash-bytes.
    if patch:
        print(
            f"  +--{FILE_HASHING_ALGO().name:10}({len(hash)}bytes@0x{fd.tell():08x}):"
            f" {hash.hex()}"
        )
        fd.write(hash)
    else:
        stored = fd.read()
        if hash != stored:
            print(
                f"{fd.name}: image's hash({stored.hex()}) != {hash.hex()}!",
                file=sys.stderr,
            )


Image = namedtuple("Image", "nsegments, is_hashed, checksum, hash, infos")


def load_and_verify_image(fd: BinaryIO) -> Image:
    """Trimmed down from *esptool.py* and reading the specs."""
    fd.seek(0x00)
    (header_magic, nsegments, is_hashed) = struct.unpack("BB21xB", fd.read(24))
    if header_magic != ESP_IMAGE_HEADER_MAGIC:
        print(
            f"{fd.name}: image's ESP_IMAGE_HEADER_MAGIC(0x{header_magic:02x}) != "
            f"0x{ESP_IMAGE_HEADER_MAGIC:02x}!",
            file=sys.stderr,
        )

    fd.seek(0x20)
    app_magic = struct.unpack("<I", fd.read(4))[0]
    if app_magic != ESP_APP_DESC_MAGIC_WORD:
        print(
            f"E{fd.name}: image's ESP_APP_DESC_MAGIC_WORD(0x{app_magic:02x}) != "
            f"0x{ESP_APP_DESC_MAGIC_WORD:02x}!",
            file=sys.stderr,
        )

    chk = checksum_image(fd, nsegments)

    hash = ""
    if is_hashed:
        hash = hash_image(fd)

    fd.seek(ESP_APP_DESC_OFFSET)
    (appver, name, time, date) = struct.unpack("32s 32s 16s 16s", fd.read(96))
    return Image(nsegments, is_hashed, chk, hash, AppInfos(name, appver, date, time))


#%%


def patch_bytestring(
    fd: BinaryIO, label: str, seek: int, length: int, data: str, source: str
):
    bdata = struct.pack(f"{length}sb", data.encode(ENCODING), 0)
    print(
        f"  +--{label:10}({length + 1}bytes@0x{seek:08x} from {source:14}): |{data.strip()}|"
    )
    fd.seek(seek)
    fd.write(bdata)


def patch_bytestring_with_infos(
    fd: BinaryIO, app_infos: AppInfos, sources: AppInfos
) -> bool:
    if any(app_infos):
        if app_infos.appver:
            patch_bytestring(fd, "app_ver", 0x30, 30, app_infos.appver, sources.appver)
        if app_infos.appname:
            patch_bytestring(
                fd, "app_name", 0x50, 30, app_infos.appname, sources.appname
            )
        if app_infos.date or app_infos.time:
            patch_bytestring(fd, "build_time", 0x70, 14, app_infos.time, sources.time)
            patch_bytestring(fd, "build_date", 0x80, 14, app_infos.date, sources.date)

        return True
    else:
        print("  +--no image-infos enabled to patch!")


def patch_app_infos(img_fpath, app_infos: AppInfos, sources: AppInfos):
    print(f"Patching app-infos --> {img_fpath}:")
    with io.open(img_fpath, "r+b") as fd:
        img = load_and_verify_image(fd)
        is_patched = patch_bytestring_with_infos(fd, app_infos, sources)
        if is_patched:
            checksum_image(fd, img.nsegments, patch=True)
            if img.is_hashed:
                hash_image(fd, patch=True)


def PatchAppInfos(source, target, env):
    img_fpath = source[0].path
    ## DEBUG: keep a copy of unpatched image.
    # shutil.copy(img_fpath, img_fpath + ".OK")
    app_infos, sources = _collect_app_infos(env)
    patch_app_infos(img_fpath, app_infos, sources)


def install_patch_app_infos(env):
    patch_action = env.AddCustomTarget(
        "patchappinfos",
        "$BUILD_DIR/${PROGNAME}.bin",
        PatchAppInfos,
        title="Engarve image with app-infos",
        description="Patch `${PROGNAME}.bin` with project name/version & build timestamp",
        always_build=True,
    )
    env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", patch_action)
    env.Depends(target="upload", dependency=patch_action)

    def DumpAppInfos(source, target, env):
        img_fpath = source[0].path
        with io.open(img_fpath, "rb") as fd:
            img = load_and_verify_image(fd)

        print(img.infos._asdict())

    env.Default(patch_action)

    ## TODO: make DumpAppInfos a new CLI
    patch_action = env.AddCustomTarget(
        "appinfos",
        "$BUILD_DIR/${PROGNAME}.bin",
        DumpAppInfos,
        title="Dump app-infos in image",
        description="Dump project name/version & build timestamp engraved in `${PROGNAME}.bin`",
        always_build=True,
    )


if __name__ == "SCons.Script":
    Import("env")

    install_patch_app_infos(env)
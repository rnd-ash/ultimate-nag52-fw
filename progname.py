"""
A *platformio* PRE-script to rename output elf/bin program binaries
based on a cascade-fallback list of custom-options & commands to derive them
from as `<project-name>-<project-version>`:
- **project-name**:
  - `custom_prog_name` custom-option in `platformio.ini`
  - last element of project dir
- **project-version**:
  - `custom_prog_version` custom-option in `platformio.ini`
  - `${PROJECT_PATH}/version.txt` file in project root
  - git-describe
  - `0.0.0`
See https://docs.platformio.org/en/stable/scripting/examples/custom_program_name.html
"""
import io
import subprocess

from typing import Tuple


#: The custom-option that when defined, modifies the program-version.
PROG_NAME_OPTION = "custom_prog_name"
#: The custom-option that when defined, modifies the program-version.
PROG_VERSION_OPTION = "custom_prog_version"
#: A filepath to read the version of the program from.
PROG_VERSION_FPATH = "${PROJECT_PATH}/version.txt"
#: The shell-command to collect the output of `git describe`.
GIT_DESCRIBE_CMD = "git describe --always --long --dirty".split()


def fallback_get(*getters: Tuple[callable, str]):
    for getter, label in getters:
        value = getter()
        if value is not None:
            return value, label


def get_program_name(env) -> Tuple[str, str]:
    """
    :return: 2-tuple of (project, source-label)
    """
    return fallback_get(
        (
            lambda: env.GetProjectOption(PROG_NAME_OPTION, None),
            "custom_option",
        ),
        (
            lambda: env.Dir(env.get("PROJECT_SRC_DIR")).get_path_elements()[-1].name,
            "project_dir",
        ),
    )


def read_version_from_file(env) -> str:
    try:
        ver_fpath = env.subst(PROG_VERSION_FPATH)
        with io.open(ver_fpath, "rt") as fd:
            return fd.read().strip()
    except Exception as ex:
        ## Logging source of each app-ingo
        pass
        # sys.stderr.write(
        #     f"could not read project-version from file '{ver_fpath}'"
        #     f" due to {type(ex).__name__}: {ex}\n"
        # )


def git_describe() -> str:
    try:
        return subprocess.check_output(
            GIT_DESCRIBE_CMD, universal_newlines=True
        ).strip()
    except Exception as ex:
        ## Logging source of each app-ingo
        pass
        # sys.stderr.write(
        #     f"could not read project-version from `{GIT_DESCRIBE_CMD}`"
        #     f" due to {type(ex).__name__}: {ex}\n"
        # )


def get_program_ver(env) -> Tuple[str, str]:
    """
    :return: 2-tuple of (project_version, source-label)
    """
    return fallback_get(
        (
            lambda: env.GetProjectOption(PROG_VERSION_OPTION, None),
            "custom_option",
        ),
        (lambda: read_version_from_file(env), "version_file"),
        (lambda: git_describe(), "git_describe"),
        (lambda: "0.0.0", "fixed"),
    )


def rename_progname(env):
    project_name, project_name_src = get_program_name(env)
    project_ver, project_ver_src = get_program_ver(env)
    progname = f"{project_name}-{project_ver}"
    print(
        f"RENAME: '{env['PROGNAME']}' --> '{progname}'"
        f"\n  sourced from project_name({project_name_src}), project_ver({project_ver_src})"
    )

    env.Replace(PROGNAME=progname)


if __name__ == "SCons.Script":
    Import("env")

    rename_progname(env)
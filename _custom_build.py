import os
import shutil
import sys
from pathlib import Path
from setuptools import Extension
from setuptools.command.build_ext import build_ext
from setuptools.command.build_py import build_py as build_py
import multiprocessing

try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel


    class bdist_wheel(_bdist_wheel):
        def finalize_options(self):
            """we have executable binaries"""
            _bdist_wheel.finalize_options(self)
            self.root_is_pure = False

        def get_tag(self):
            """build for specific ABI and platform"""
            python, abi, plat = _bdist_wheel.get_tag(self)
            return python, abi, plat
except ImportError:
    bdist_wheel = None


class CMakeExtension(Extension):
    def __init__(self, name):
        # don't invoke the original build_ext for this special extension
        super().__init__(name, sources=[])


class BuildPy(build_py):
    def run(self):
        self.run_command("build_ext")
        return super().run()

    def initialize_options(self):
        super().initialize_options()
        if self.distribution.ext_modules is None:
            self.distribution.ext_modules = [CMakeExtension('src')]


class BuildCMakeExt(build_ext):
    def run(self):
        for ext in self.extensions:
            self.build_cmake(ext)

    def build_cmake(self, ext):
        cwd = Path().absolute()

        # these dirs will be created in build_py, so if you don't have
        # any python sources to bundle, the dirs will be missing
        build_temp = Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)

        config = 'Debug' if self.debug else 'Release'
        num_cpus = 4
        try:
            num_cpus = multiprocessing.cpu_count()
        except NotImplementedError:
            pass
        
        if os.name == "posix":
            CMAKE_GENERATOR = "Unix Makefiles"
            cmake_args = [
                '-G', CMAKE_GENERATOR,
                f'-DCMAKE_BUILD_TYPE={config}',
                str(Path(ext.name).absolute())  # src directory path
            ]
            build_args = [
                '--config', config,
                '--', f'-j{num_cpus}'
            ]
        elif os.name == "nt":
            # On Windows, use Ninja or Visual Studio generator
            # Ninja is preferred for better compatibility with cibuildwheel
            CMAKE_GENERATOR = "Ninja"
            cmake_args = [
                '-G', CMAKE_GENERATOR,
                f'-DCMAKE_BUILD_TYPE={config}',
                str(Path(ext.name).absolute())  # src directory path
            ]
            build_args = [
                '--config', config,
                '--parallel', str(num_cpus)
            ]
        else:
            print("Unsupported OS: " + os.name)
            sys.exit(1)

        os.chdir(str(build_temp))
        self.spawn(['cmake', str(cwd)] + cmake_args)
        if not self.dry_run:
            self.spawn(['cmake', '--build', '.'] + build_args)
        # Troubleshooting: if fail on line above then delete all possible
        # temporary CMake files including "CMakeCache.txt" in top level dir.
        os.chdir(str(cwd))
        
        # Copy built binaries
        bin_dest = Path(self.build_lib) / 'kstar_planner' / 'builds' / 'release' / 'bin'
        shutil.copytree(build_temp / 'bin', bin_dest, dirs_exist_ok=True)
        
        # On Windows with MinGW, copy required runtime DLLs
        if os.name == "nt":
            self._copy_mingw_dlls(bin_dest)
    
    def _copy_mingw_dlls(self, bin_dest):
        """Copy MinGW runtime DLLs to the bin directory on Windows."""
        import subprocess
        
        # Find the compiler executable
        try:
            result = subprocess.run(['where', 'g++'], capture_output=True, text=True, check=True)
            gcc_path = Path(result.stdout.strip().split('\n')[0])
            mingw_bin = gcc_path.parent
            
            # List of required DLLs
            required_dlls = [
                'libstdc++-6.dll',
                'libgcc_s_seh-1.dll',
                'libgcc_s_dw2-1.dll',  # 32-bit alternative
                'libwinpthread-1.dll'
            ]
            
            copied_count = 0
            for dll_name in required_dlls:
                dll_path = mingw_bin / dll_name
                if dll_path.exists():
                    dest_path = bin_dest / dll_name
                    shutil.copy2(dll_path, dest_path)
                    print(f"Copied {dll_name} to {dest_path}")
                    copied_count += 1
            
            if copied_count > 0:
                print(f"Successfully copied {copied_count} MinGW runtime DLL(s)")
            else:
                print("Warning: No MinGW DLLs found. If using MSVC, this is expected.")
                
        except (subprocess.CalledProcessError, FileNotFoundError, IndexError):
            print("Warning: Could not locate MinGW compiler. Skipping DLL copy.")
            print("If the executable fails to run, manually copy MinGW DLLs to the bin directory.")
